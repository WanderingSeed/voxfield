#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/mesh_pcl.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {
class SimpleTsdfVisualizer {
 public:
  explicit SimpleTsdfVisualizer(const rclcpp::Node::SharedPtr& node)
    : node_(node),
        tsdf_surface_distance_threshold_factor_(2.0),
        tsdf_world_frame_("world"),
        tsdf_mesh_color_mode_(ColorMode::kColor),
        tsdf_voxel_ply_output_path_("") {
  RCLCPP_DEBUG(node_->get_logger(), "Setting up ROS2 publishers...");

  surface_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "tsdf_voxels_near_surface", rclcpp::QoS(1).transient_local());

  tsdf_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "all_tsdf_voxels", rclcpp::QoS(1).transient_local());

  mesh_pub_ = node_->create_publisher<voxblox_msgs::msg::Mesh>("mesh", rclcpp::QoS(1).transient_local());

  mesh_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "mesh_as_pointcloud", rclcpp::QoS(1).transient_local());

  mesh_pcl_mesh_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "mesh_pcl", rclcpp::QoS(1).transient_local());

  RCLCPP_DEBUG(node_->get_logger(), "Retrieving ROS2 parameters...");

  node_->declare_parameter("tsdf_surface_distance_threshold_factor", tsdf_surface_distance_threshold_factor_);
  node_->get_parameter("tsdf_surface_distance_threshold_factor", tsdf_surface_distance_threshold_factor_);
  node_->declare_parameter("tsdf_world_frame", tsdf_world_frame_);
  node_->get_parameter("tsdf_world_frame", tsdf_world_frame_);
  node_->declare_parameter("tsdf_voxel_ply_output_path", tsdf_voxel_ply_output_path_);
  node_->get_parameter("tsdf_voxel_ply_output_path", tsdf_voxel_ply_output_path_);
  node_->declare_parameter("tsdf_mesh_output_path", tsdf_mesh_output_path_);
  node_->get_parameter("tsdf_mesh_output_path", tsdf_mesh_output_path_);

  std::string color_mode = "color";
  node_->declare_parameter("tsdf_mesh_color_mode", color_mode);
  node_->get_parameter("tsdf_mesh_color_mode", color_mode);
    if (color_mode == "color") {
      tsdf_mesh_color_mode_ = ColorMode::kColor;
    } else if (color_mode == "height") {
      tsdf_mesh_color_mode_ = ColorMode::kHeight;
    } else if (color_mode == "normals") {
      tsdf_mesh_color_mode_ = ColorMode::kNormals;
    } else if (color_mode == "lambert") {
      tsdf_mesh_color_mode_ = ColorMode::kLambert;
    } else if (color_mode == "gray") {
      tsdf_mesh_color_mode_ = ColorMode::kGray;
    } else {
      RCLCPP_FATAL(
          node_->get_logger(), "Undefined mesh coloring mode: %s",
          color_mode.c_str());
      rclcpp::shutdown();
    }

    // no-op for ROS2; parameters already declared and node handles created.
  }

  void run(const Layer<TsdfVoxel>& tsdf_layer);

 private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_pointcloud_pub_;
  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mesh_pointcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pcl_mesh_pub_;

  // Settings
  double tsdf_surface_distance_threshold_factor_;
  std::string tsdf_world_frame_;
  ColorMode tsdf_mesh_color_mode_;
  std::string tsdf_voxel_ply_output_path_;
  std::string tsdf_mesh_output_path_;
};

void SimpleTsdfVisualizer::run(const Layer<TsdfVoxel>& tsdf_layer) {
  RCLCPP_INFO(
      node_->get_logger(),
      "\tTSDF Layer info:\n \tVoxel size:\t\t %f \n \t# Voxels per side:\t %ld "
      "\n \tMemory size:\t\t %ld MB\n \t# Allocated blocks:\t %ld \n",
      tsdf_layer.voxel_size(), tsdf_layer.voxels_per_side(),
      tsdf_layer.getMemorySize() / 1024 / 1024,
      tsdf_layer.getNumberOfAllocatedBlocks());

  RCLCPP_DEBUG(node_->get_logger(), "Visualize voxels near surface...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    const FloatingPoint surface_distance_thresh_m =
        tsdf_layer.voxel_size() * tsdf_surface_distance_threshold_factor_;
    voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
        tsdf_layer, surface_distance_thresh_m, &pointcloud);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    msg.header.frame_id = tsdf_world_frame_;
    msg.header.stamp = rclcpp::Clock().now();
    surface_pointcloud_pub_->publish(msg);
  }

  RCLCPP_DEBUG(node_->get_logger(), "\tVisualize all voxels...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    voxblox::createDistancePointcloudFromTsdfLayer(tsdf_layer, &pointcloud);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    msg.header.frame_id = tsdf_world_frame_;
    msg.header.stamp = rclcpp::Clock().now();
    tsdf_pointcloud_pub_->publish(msg);

    if (!tsdf_voxel_ply_output_path_.empty()) {
      pcl::PLYWriter writer;
      constexpr bool kUseBinary = true;
      writer.write(tsdf_voxel_ply_output_path_, pointcloud, kUseBinary);
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "\tVisualize mesh...");
  {
    std::shared_ptr<MeshLayer> mesh_layer;
    mesh_layer.reset(new MeshLayer(tsdf_layer.block_size()));
    MeshIntegratorConfig mesh_config;
    std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator;
    mesh_integrator.reset(new MeshIntegrator<TsdfVoxel>(
        mesh_config, tsdf_layer, mesh_layer.get()));

    constexpr bool kOnlyMeshUpdatedBlocks = false;
    constexpr bool kClearUpdatedFlag = false;
    mesh_integrator->generateMesh(kOnlyMeshUpdatedBlocks, kClearUpdatedFlag);

    // Output as native voxblox mesh.
    voxblox_msgs::msg::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer, tsdf_mesh_color_mode_, &mesh_msg);
    mesh_msg.header.frame_id = tsdf_world_frame_;
    mesh_msg.header.stamp = rclcpp::Clock().now();
    mesh_pub_->publish(mesh_msg);

    // Output as point cloud.
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    fillPointcloudWithMesh(mesh_layer, tsdf_mesh_color_mode_, &pointcloud);
    sensor_msgs::msg::PointCloud2 pcmsg;
    pcl::toROSMsg(pointcloud, pcmsg);
    pcmsg.header.frame_id = tsdf_world_frame_;
    pcmsg.header.stamp = rclcpp::Clock().now();
    mesh_pointcloud_pub_->publish(pcmsg);

    // Output as pcl mesh.
    pcl::PolygonMesh polygon_mesh;
    toConnectedPCLPolygonMesh(*mesh_layer, tsdf_world_frame_, &polygon_mesh);
    visualization_msgs::msg::Marker pcl_mesh_msg;
    pcl_mesh_msg.header.frame_id = tsdf_world_frame_;
    pcl_mesh_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    pcl_mesh_msg.action = visualization_msgs::msg::Marker::ADD;
    // ... 设置姿态、比例、颜色等（如 marker.color.a = 1.0; marker.color.r =
    // 0.5; ...）
    // 1. 将 mesh 中的点云数据转换为 pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(
        polygon_mesh.cloud, *cloud);  // 注意PCL 1.8+使用此函数[citation:1]
    // 2. 遍历所有多边形（三角形）
    for (const auto& polygon : polygon_mesh.polygons) {
      // 假设 polygon.vertices 有3个索引，构成一个三角形
      if (polygon.vertices.size() == 3) {
        for (const auto& vertex_index : polygon.vertices) {
          const auto& point = cloud->points[vertex_index];
          geometry_msgs::msg::Point p;
          p.x = point.x;
          p.y = point.y;
          p.z = point.z;
          pcl_mesh_msg.points.push_back(p);
          // 可以为每个点设置颜色，或使用 marker.colors 数组
        }
      }
    }

    mesh_pcl_mesh_pub_->publish(pcl_mesh_msg);

    if (!tsdf_mesh_output_path_.empty()) {
      if (voxblox::outputMeshLayerAsPly(tsdf_mesh_output_path_, *mesh_layer)) {
        RCLCPP_INFO(
            node_->get_logger(),
            "Output mesh PLY file to %s", tsdf_mesh_output_path_.c_str());
      }
    }
  }
}

}  // namespace voxblox

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto node = rclcpp::Node::make_shared("visualize_tsdf_node");

  std::string tsdf_proto_path = "";
  node->declare_parameter("tsdf_proto_path", tsdf_proto_path);
  node->get_parameter("tsdf_proto_path", tsdf_proto_path);
  if (tsdf_proto_path.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Please provide a TSDF proto file via parameter: tsdf_proto_path");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Visualize TSDF grid from %s", tsdf_proto_path.c_str());

  RCLCPP_INFO(node->get_logger(), "Loading...");
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer;
  if (!voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_proto_path, &tsdf_layer)) {
    RCLCPP_FATAL(node->get_logger(), "Unable to load a TSDF grid from: %s", tsdf_proto_path.c_str());
    return 1;
  }
  CHECK(tsdf_layer);
  RCLCPP_INFO(node->get_logger(), "Done.");

  RCLCPP_INFO(node->get_logger(), "Visualizing...");
  voxblox::SimpleTsdfVisualizer visualizer(node);
  visualizer.run(*tsdf_layer);
  RCLCPP_INFO(node->get_logger(), "Done.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
