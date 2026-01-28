#include <deque>
#include <gflags/gflags.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/occupancy_tsdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

// This binary evaluates a pre-built voxblox map against a provided ground
// truth dataset, provided as pointcloud, and outputs a variety of
// statistics.
namespace voxblox {

class VoxbloxEvaluator {
 public:
  VoxbloxEvaluator(
      const rclcpp::Node::SharedPtr& node, const rclcpp::Node::SharedPtr& node_private);
  void evaluate();
  void visualize();
  void evaluateEsdf();
  void visualizeEsdf();
  void generatePointCloudFromOcc();
  bool shouldExit() const {
    return !visualize_;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr node_private_;

  // Whether to do the visualizations (involves generating mesh of the TSDF
  // layer) and keep alive (for visualization) after finishing the eval.
  // Otherwise only outputs the evaluations statistics.
  bool visualize_;
  // Whether to recolor the voxels by error to the GT before generating a mesh.
  bool recolor_by_error_;
  // If visualizing, what TF frame to visualize in.
  std::string frame_id_;
  // Whether to also evaluate the Esdf mapping accuracy
  bool eval_esdf_;
  // Whether to use the occupied grid centers as the reference for evaluating
  // esdf.
  bool use_occ_ref_esdf_;
  // How to color the mesh.
  ColorMode color_mode_;
  // esdf & tsdf slice level (unit: m)
  float slice_level_;
  // error limit for visualization (unit: m)
  float error_limit_m_;
  // evaluate only the voxels with positive ESDF
  bool eval_only_positive_;

  // Transformation between the ground truth dataset and the voxblox map.
  // The GT is transformed INTO the voxblox coordinate frame.
  Transformation T_V_G_;

  // Visualization publishers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gt_ptcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_error_slice_pub_;

  // Core data to compare.
  std::shared_ptr<Layer<TsdfVoxel>> tsdf_layer_;
  std::shared_ptr<Layer<EsdfVoxel>> esdf_layer_;
  std::shared_ptr<Layer<OccupancyVoxel>> occ_layer_;
  pcl::PointCloud<pcl::PointXYZRGB> gt_ptcloud_;
  pcl::PointCloud<pcl::PointXYZRGB> occ_ptcloud_;

  // Interpolator to get the distance at the exact point in the GT.
  Interpolator<TsdfVoxel>::Ptr interpolator_;

  // Occupancy grid from Tsdf map
  std::unique_ptr<OccTsdfIntegrator> occupancy_integrator_;

  // Mesh visualization.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
};

VoxbloxEvaluator::VoxbloxEvaluator(
    const rclcpp::Node::SharedPtr& node, const rclcpp::Node::SharedPtr& node_private)
    : node_(node),
      node_private_(node_private),
      visualize_(true),
      recolor_by_error_(false),
      frame_id_("world"),
      eval_esdf_(false),
      use_occ_ref_esdf_(true),
      slice_level_(1.0),
      error_limit_m_(0.2),
      eval_only_positive_(false) {
  // Load parameters.
  node_private_->declare_parameter("visualize", visualize_);
  node_private_->get_parameter("visualize", visualize_);
  node_private_->declare_parameter("recolor_by_error", recolor_by_error_);
  node_private_->get_parameter("recolor_by_error", recolor_by_error_);
  node_private_->declare_parameter("frame_id", frame_id_);
  node_private_->get_parameter("frame_id", frame_id_);
  node_private_->declare_parameter("eval_esdf", eval_esdf_);
  node_private_->get_parameter("eval_esdf", eval_esdf_);
  node_private_->declare_parameter("use_occ_ref", use_occ_ref_esdf_);
  node_private_->get_parameter("use_occ_ref", use_occ_ref_esdf_);
  node_private_->declare_parameter("slice_level", slice_level_);
  node_private_->get_parameter("slice_level", slice_level_);
  node_private_->declare_parameter("error_limit_m", error_limit_m_);
  node_private_->get_parameter("error_limit_m", error_limit_m_);
  node_private_->declare_parameter("eval_only_positive", eval_only_positive_);
  node_private_->get_parameter("eval_only_positive", eval_only_positive_);

  // Load transformations.
  node_private_->declare_parameter("T_V_G", rclcpp::PARAMETER_DOUBLE_ARRAY);
  std::vector<double> T_V_G_vector;
  if (node_private_->get_parameter("T_V_G", T_V_G_vector)) {
    Eigen::Matrix4d T_V_G_matrix;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        T_V_G_matrix(i, j) = T_V_G_vector[i * 4 + j];
      }
    }
    T_V_G_ = Transformation(Eigen::Matrix4f(T_V_G_matrix.cast<float>()));

    bool invert_static_tranform = false;
    node_private_->declare_parameter("invert_T_V_G", invert_static_tranform);
    node_private_->get_parameter("invert_T_V_G", invert_static_tranform);
    if (invert_static_tranform) {
      T_V_G_ = T_V_G_.inverse();
    }
  }

  // Load the actual map and GT.
  // Just exit if there's any issues here (this is just an evaluation node,
  // after all).
  std::string voxblox_file_path, gt_file_path;
  std::string voxblox_esdf_file_path, voxblox_occ_file_path;
  node_private_->declare_parameter("voxblox_file_path", std::string());
  node_private_->get_parameter("voxblox_file_path", voxblox_file_path);
  CHECK(!voxblox_file_path.empty()) << "No file path provided for voxblox map! Set the \"voxblox_file_path\" param.";
  node_private_->declare_parameter("gt_file_path", std::string());
  node_private_->get_parameter("gt_file_path", gt_file_path);
  CHECK(!gt_file_path.empty()) << "No file path provided for ground truth pointcloud! Set the \"gt_file_path\" param.";
  if (eval_esdf_) {
    node_private_->declare_parameter("voxblox_esdf_file_path", std::string());
    node_private_->get_parameter("voxblox_esdf_file_path", voxblox_esdf_file_path);
    CHECK(!voxblox_esdf_file_path.empty()) << "No file path provided for voxblox esdf map! Set the \"voxblox_esdf_file_path\" param.";
    if (use_occ_ref_esdf_) {
      node_private_->declare_parameter("voxblox_occ_file_path", std::string());
      node_private_->get_parameter("voxblox_occ_file_path", voxblox_occ_file_path);
      CHECK(!voxblox_occ_file_path.empty()) << "No file path provided for voxblox occ map! Set the \"voxblox_occ_file_path\" param.";
    }
  }

  CHECK(io::LoadLayer<TsdfVoxel>(voxblox_file_path, &tsdf_layer_))
      << "Could not load voxblox map.";
  if (eval_esdf_) {
    CHECK(io::LoadLayer<EsdfVoxel>(voxblox_esdf_file_path, &esdf_layer_))
        << "Could not load voxblox esdf map.";
    if (use_occ_ref_esdf_) {
      CHECK(io::LoadLayer<OccupancyVoxel>(voxblox_occ_file_path, &occ_layer_))
          << "Could not load voxblox occupancy map.";
    }
  }
  pcl::PLYReader ply_reader;

  CHECK_EQ(ply_reader.read(gt_file_path, gt_ptcloud_), 0)
      << "Could not load pointcloud ground truth.";

  // Initialize the interpolator.
  interpolator_.reset(new Interpolator<TsdfVoxel>(tsdf_layer_.get()));

  // Set up Occupancy map and integrator (deprecated, since we directly load the
  // map) occ_layer_.reset(new Layer<OccupancyVoxel>(tsdf_layer_->voxel_size(),
  //                                            tsdf_layer_->voxels_per_side()));
  // OccTsdfIntegrator::Config occ_tsdf_integrator_config;
  // occupancy_integrator_.reset(new OccTsdfIntegrator(
  //     occ_tsdf_integrator_config, tsdf_layer_.get(), occ_layer_.get()));

  // If doing visualizations, initialize the publishers.
  if (visualize_) {
    mesh_pub_ = node_private_->create_publisher<visualization_msgs::msg::MarkerArray>("mesh", rclcpp::QoS(1).transient_local());
    gt_ptcloud_pub_ = node_private_->create_publisher<sensor_msgs::msg::PointCloud2>("gt_ptcloud", rclcpp::QoS(1).transient_local());
    esdf_error_slice_pub_ = node_private_->create_publisher<sensor_msgs::msg::PointCloud2>("esdf_error_slice", rclcpp::QoS(1).transient_local());
    std::string color_mode("color");
    node_private_->declare_parameter("color_mode", color_mode);
    node_private_->get_parameter("color_mode", color_mode);
    if (color_mode == "color") {
      color_mode_ = ColorMode::kColor;
    } else if (color_mode == "height") {
      color_mode_ = ColorMode::kHeight;
    } else if (color_mode == "normals") {
      color_mode_ = ColorMode::kNormals;
    } else if (color_mode == "lambert") {
      color_mode_ = ColorMode::kLambert;
    } else {  // Default case is gray.
      color_mode_ = ColorMode::kGray;
    }
  }

  std::cout << "Evaluating " << voxblox_file_path << std::endl;
}

void VoxbloxEvaluator::evaluate() {
  // First, transform the pointcloud into the correct coordinate frame.
  // First, rotate the pointcloud into the world frame.
  pcl::transformPointCloud(
      gt_ptcloud_, gt_ptcloud_, T_V_G_.getTransformationMatrix());

  // Go through each point, use trilateral interpolation to figure out the
  // distance at that point.
  // This double-counts -- multiple queries to the same voxel will be counted
  // separately.
  uint64_t total_evaluated_voxels = 0;
  uint64_t unknown_voxels = 0;
  uint64_t outside_truncation_voxels = 0;

  // TODO(helenol): make this dynamic.
  double truncation_distance = 2 * tsdf_layer_->voxel_size();

  double mse = 0.0, mae = 0.0;

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it =
           gt_ptcloud_.begin();
       it != gt_ptcloud_.end(); ++it) {
    Point point(it->x, it->y, it->z);

    FloatingPoint distance = 0.0;
    float weight = 0.0;
    bool valid = false;

    const float min_weight = 0.01;
    const bool interpolate = true;
    // We will do multiple lookups -- the first is to determine whether the
    // voxel exists.
    if (!interpolator_->getNearestDistanceAndWeight(
            point, &distance, &weight)) {
      unknown_voxels++;
    } else if (weight <= min_weight) {
      unknown_voxels++;
    } else if (distance >= truncation_distance) {
      outside_truncation_voxels++;
      mse += truncation_distance * truncation_distance;
      mae += truncation_distance;
      valid = true;
    } else {
      // In case this fails, distance is still the nearest neighbor distance.
      interpolator_->getDistance(point, &distance, interpolate);
      mse += distance * distance;
      mae += std::abs(distance);
      valid = true;
    }

    if (valid && visualize_ && recolor_by_error_) {
      Layer<TsdfVoxel>::BlockType::Ptr block_ptr =
          tsdf_layer_->getBlockPtrByCoordinates(point);
      if (block_ptr != nullptr) {
        TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(point);
        voxel.color = grayColorMap(std::fabs(distance) / truncation_distance);
      }
    }

    total_evaluated_voxels++;
  }

  double rms = sqrt(mse / (total_evaluated_voxels - unknown_voxels));
  mae /= (total_evaluated_voxels - unknown_voxels);

  std::cout << "Finished evaluating.\n"
            << "\nRMSE(m):             " << rms
            << "\nMAE(m):              " << mae
            << "\nTotal evaluated:     " << total_evaluated_voxels
            << "\nUnknown voxels:       " << unknown_voxels << " ("
            << static_cast<double>(unknown_voxels) / total_evaluated_voxels
            << ")\nOutside truncation: " << outside_truncation_voxels << " ("
            << outside_truncation_voxels /
                   static_cast<double>(total_evaluated_voxels)
            << ")\n";

  if (eval_esdf_) {
    if (use_occ_ref_esdf_) {
      generatePointCloudFromOcc();
    }
    evaluateEsdf();
  }

  if (visualize_) {
    visualize();
    if (eval_esdf_) {
      visualizeEsdf();
    }
  }
}

void VoxbloxEvaluator::evaluateEsdf() {
  // build the kd tree of the ground truth or occupied grid point cloud

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  std::cout << "Begin to evaluate ESDF mapping accuracy ";
  if (use_occ_ref_esdf_) {
    kdtree.setInputCloud(occ_ptcloud_.makeShared());
    std::cout << "based on the occupied grid centers.\n";
  } else {
    kdtree.setInputCloud(gt_ptcloud_.makeShared());
    std::cout << "based on the ground truth point cloud.\n";
  }

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  BlockIndexList esdf_blocks;
  esdf_layer_->getAllAllocatedBlocks(&esdf_blocks);

  double mse = 0.0, mae = 0.0;
  uint64_t total_evaluated_voxels = 0;

  for (const BlockIndex& block_index : esdf_blocks) {
    Block<EsdfVoxel>::ConstPtr esdf_block =
        esdf_layer_->getBlockPtrByIndex(block_index);
    if (!esdf_block) {
      continue;
    }

    const size_t num_voxels_per_block = esdf_block->num_voxels();
    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const EsdfVoxel& esdf_voxel =
          esdf_block->getVoxelByLinearIndex(lin_index);
      if (!esdf_voxel.observed)
        continue;
      if (eval_only_positive_ && esdf_voxel.distance < 0)
        continue;

      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      GlobalIndex global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(
          block_index, voxel_index, esdf_layer_->voxels_per_side());

      Point point =
          getCenterPointFromGridIndex(global_index, esdf_layer_->voxel_size());

      kdtree.nearestKSearch(
          pcl::PointXYZRGB(point(0), point(1), point(2)), 1, pointIdxNKNSearch,
          pointNKNSquaredDistance);
      float cur_gt_dist = std::sqrt(pointNKNSquaredDistance[0]);
      float cur_est_dist = std::abs(esdf_voxel.distance);
      float cur_error_dist = cur_est_dist - cur_gt_dist;
      mse += (cur_error_dist * cur_error_dist);
      mae += std::abs(cur_error_dist);

      // NOTE(yuepan): It would not be used any more anyway, so we use it
      // to plot the slice. esdf_voxel is const, so try to get a new one.
      EsdfVoxel* cur_esdf_vox =
          esdf_layer_->getVoxelPtrByGlobalIndex(global_index);

      // Clamped with the error limit for visualization
      cur_esdf_vox->distance =
          std::max(-error_limit_m_, std::min(error_limit_m_, cur_error_dist));

      total_evaluated_voxels++;
    }
  }

  double rms = sqrt(mse / total_evaluated_voxels);
  mae /= total_evaluated_voxels;

  std::cout << "Finished evaluating ESDF map.\n"
            << "\nRMSE(m):           " << rms << "\nMAE(m):            " << mae
            << "\nTotal evaluated:     " << total_evaluated_voxels << "\n";
}

void VoxbloxEvaluator::generatePointCloudFromOcc() {
  // occupancy_integrator_->updateFromTsdfLayer(true, true);

  BlockIndexList occ_blocks;
  occ_layer_->getAllAllocatedBlocks(&occ_blocks);

  int voxels_per_side = occ_layer_->voxels_per_side();
  float voxel_size = occ_layer_->voxel_size();

  for (const BlockIndex& block_index : occ_blocks) {
    Block<OccupancyVoxel>::ConstPtr occ_block =
        occ_layer_->getBlockPtrByIndex(block_index);
    if (!occ_block) {
      continue;
    }
    const size_t num_voxels_per_block = occ_block->num_voxels();
    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const OccupancyVoxel& occ_voxel =
          occ_block->getVoxelByLinearIndex(lin_index);
      if (!occ_voxel.observed || occ_voxel.probability_log < 0.7)
        continue;

      VoxelIndex voxel_index =
          occ_block->computeVoxelIndexFromLinearIndex(lin_index);
      GlobalIndex global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(
          block_index, voxel_index, voxels_per_side);

      Point point = getCenterPointFromGridIndex(global_index, voxel_size);

      pcl::PointXYZRGB pt(point(0), point(1), point(2));
      occ_ptcloud_.points.push_back(pt);
    }
  }
}

void VoxbloxEvaluator::visualize() {
  // Generate the mesh.
  MeshIntegratorConfig mesh_config;
  mesh_layer_.reset(new MeshLayer(tsdf_layer_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_layer_.get(), mesh_layer_.get()));

  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);

  // Publish mesh.
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(1);
  marker_array.markers[0].header.frame_id = frame_id_;
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  mesh_pub_->publish(marker_array);

  sensor_msgs::msg::PointCloud2 pcmsg;
  pcl::toROSMsg(gt_ptcloud_, pcmsg);
  pcmsg.header.frame_id = frame_id_;
  pcmsg.header.stamp = node_->get_clock()->now();
  gt_ptcloud_pub_->publish(pcmsg);
  RCLCPP_INFO(node_->get_logger(), "Finished visualizing.");
}

// Generate a slice, colored with esdf mapping error
void VoxbloxEvaluator::visualizeEsdf() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  // TODO(yuepan): for visualization colormap, the value (here the error)
  // should evenly lay around 0.0, make sure to set the range of the
  // value for expected visualization
  createDistancePointcloudFromEsdfLayerSlice(
      *esdf_layer_, kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = frame_id_;
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    msg.header.frame_id = frame_id_;
    msg.header.stamp = node_->get_clock()->now();
    esdf_error_slice_pub_->publish(msg);
  }
}

}  // namespace voxblox

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  auto node = rclcpp::Node::make_shared("voxblox_node");
  auto node_private = node;

  voxblox::VoxbloxEvaluator eval(node, node_private);
  eval.evaluate();

  if (!eval.shouldExit()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
