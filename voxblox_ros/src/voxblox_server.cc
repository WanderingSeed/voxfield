#include "voxblox_ros/voxblox_server.h"

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

#include <pcl/kdtree/kdtree_flann.h>  // py: added

namespace voxblox {

VoxbloxServer::VoxbloxServer(const rclcpp::Node::SharedPtr& node)
    : TsdfServer(node) {
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "Constructed VoxbloxServer (ROS2)");

    EsdfMap::Config esdf_config = getEsdfMapConfigFromRosParam(node_);
    EsdfIntegrator::Config esdf_integrator_config =
        getEsdfIntegratorConfigFromRosParam(node_);

    // ADD(py): Set up Occupancy map and integrator
    OccupancyMap::Config occ_config;
    occ_config.occupancy_voxel_size = esdf_config.esdf_voxel_size;
    occ_config.occupancy_voxels_per_side = esdf_config.esdf_voxels_per_side;
    OccTsdfIntegrator::Config occ_tsdf_integrator_config;
    occupancy_map_.reset(new OccupancyMap(occ_config));
    occupancy_integrator_.reset(new OccTsdfIntegrator(
        occ_tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
        occupancy_map_->getOccupancyLayerPtr()));
    // Set up map and integrator.
    esdf_map_.reset(new EsdfMap(esdf_config));
    esdf_integrator_.reset(new EsdfIntegrator(
        esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
        esdf_map_->getEsdfLayerPtr()));

    // Get parameters.
    getServerConfigFromRosParam(node_);

    clear_sphere_for_planning_ = node_->declare_parameter(
        "clear_sphere_for_planning", clear_sphere_for_planning_);
    publish_esdf_map_ =
        node_->declare_parameter("publish_esdf_map", publish_esdf_map_);
    publish_traversable_ = node_->declare_parameter("publish_traversable",
                                                    publish_traversable_);
    traversability_radius_ = node_->declare_parameter("traversability_radius",
                                                      traversability_radius_);

    double update_esdf_every_n_sec = 1.0;
    update_esdf_every_n_sec = node_->declare_parameter(
        "update_esdf_every_n_sec", update_esdf_every_n_sec);

    bool eval_esdf_on = false;
    eval_esdf_on = node_->declare_parameter("eval_esdf_on", eval_esdf_on);

    double eval_esdf_every_n_sec = 100.0;
    eval_esdf_every_n_sec = node_->declare_parameter("eval_esdf_every_n_sec",
                                                     eval_esdf_every_n_sec);

    esdf_ready_ = false;

    // Publishers
    esdf_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "esdf_pointcloud", rclcpp::QoS(1).transient_local());
    esdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "esdf_slice", rclcpp::QoS(1).transient_local());
    traversable_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "traversable", rclcpp::QoS(1).transient_local());
    esdf_error_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "esdf_error_slice", rclcpp::QoS(1).transient_local());

    esdf_map_pub_ = node_->create_publisher<voxblox_msgs::msg::Layer>(
        "esdf_map_out", rclcpp::QoS(1));

    // Subscriptions
    esdf_map_sub_ = node_->create_subscription<voxblox_msgs::msg::Layer>(
        "esdf_map_in", rclcpp::QoS(1),
        [this](const voxblox_msgs::msg::Layer::SharedPtr msg) {
          this->esdfMapCallback(msg);
        });

    // Service
    generate_esdf_srv_ = node_->create_service<std_srvs::srv::Empty>(
        "generate_esdf",
        std::bind(&VoxbloxServer::generateEsdfCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    save_esdf_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
        "save_esdf_map",
        [this](const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> req,
               std::shared_ptr<voxblox_msgs::srv::FilePath::Response> res) {
          res->success = this->saveMap(req->file_path);
        });

    // Timers
    if (update_esdf_every_n_sec > 0.0) {
      update_esdf_timer_ = node_->create_wall_timer(
          std::chrono::duration<double>(update_esdf_every_n_sec),
          std::bind(&VoxbloxServer::updateEsdfEvent, this));
    } else {
      update_esdf_every_n_ = static_cast<int>(-1.0 * update_esdf_every_n_sec);
    }

    if (eval_esdf_every_n_sec > 0.0 && eval_esdf_on) {
      eval_esdf_timer_ = node_->create_wall_timer(
          std::chrono::duration<double>(eval_esdf_every_n_sec),
          std::bind(&VoxbloxServer::evalEsdfEvent, this));
    }
  }
}

void VoxbloxServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pointcloud, msg);
  esdf_pointcloud_pub_->publish(msg);
}

void VoxbloxServer::publishSlices() {
  TsdfServer::publishSlices();

  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  createDistancePointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    esdf_slice_pub_->publish(msg);
  }
}

void VoxbloxServer::generateEsdfCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  const bool clear_esdf = true;
  if (clear_esdf) {
    esdf_integrator_->updateFromTsdfLayerBatch();
  } else {
    const bool clear_updated_flag = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
  }
  publishAllUpdatedEsdfVoxels();
  publishSlices();
}

void VoxbloxServer::updateEsdfEvent() {
  updateEsdf();
  if (publish_slices_)
    publishSlices();
}



void VoxbloxServer::publishPointclouds() {
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }

  TsdfServer::publishPointclouds();
}

void VoxbloxServer::publishTraversable() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(
      esdf_map_->getEsdfLayer(), traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  {
    sensor_msgs::msg::PointCloud2 msg3;
    pcl::toROSMsg(pointcloud, msg3);
    traversable_pub_->publish(msg3);
  }
}

void VoxbloxServer::publishMap(bool reset_remote_map) {
  if (!publish_esdf_map_) {
    return;
  }

  size_t subscribers = this->esdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_esdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_esdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(
        this->esdf_map_->getEsdfLayer(), only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    esdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_esdf_map_ = subscribers;
  TsdfServer::publishMap();
}

bool VoxbloxServer::saveMap(const std::string& file_path) {
  // Output TSDF map first, then ESDF.
  // const bool success = TsdfServer::saveMap(file_path);
  bool success = true;
  constexpr bool kClearFile = false;
  return success &&
         io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool VoxbloxServer::loadMap(const std::string& file_path) {
  // Load in the same order: TSDF first, then ESDF.
  bool success = TsdfServer::loadMap(file_path);

  constexpr bool kMultipleLayerSupport = true;
  return success &&
         io::LoadBlocksFromFile(
             file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
             kMultipleLayerSupport, esdf_map_->getEsdfLayerPtr());
}

void VoxbloxServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
    esdf_ready_ = true;
  }
}

void VoxbloxServer::updateEsdfBatch(bool full_euclidean) {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    esdf_integrator_->setFullEuclidean(full_euclidean);
    esdf_integrator_->updateFromTsdfLayerBatch();
  }
}

float VoxbloxServer::getEsdfMaxDistance() const {
  return esdf_integrator_->getEsdfMaxDistance();
}

void VoxbloxServer::setEsdfMaxDistance(float max_distance) {
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float VoxbloxServer::getTraversabilityRadius() const {
  return traversability_radius_;
}

void VoxbloxServer::setTraversabilityRadius(float traversability_radius) {
  traversability_radius_ = traversability_radius;
}

void VoxbloxServer::newPoseCallback(const Transformation& T_G_C) {
  // if update_esdf_every_n_sec_ is negative
  // we regard it as the update interval
  if (update_esdf_every_n_ > 0 && frame_count_ != 0 &&
      frame_count_ % update_esdf_every_n_ == 0) {
    updateEsdf();
    if (publish_slices_)
      publishSlices();
  }

  if (clear_sphere_for_planning_) {
    esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
  }

  // timing::Timer block_remove_timer("remove_distant_blocks");
  esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  // block_remove_timer.Stop();
}

void VoxbloxServer::esdfMapCallback(const std::shared_ptr<voxblox_msgs::msg::Layer> layer_msg) {
  timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10,
                        "Got an invalid ESDF map message!");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got an ESDF map from ROS topic!");
    if (publish_pointclouds_) {
      publishPointclouds();
    }
  }
}

void VoxbloxServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

// ADD(py):
// incrementally update occupancy map from the updated TSDF map
void VoxbloxServer::updateOccFromTsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_tsdf = true;
    const bool in_batch = true;

    // set update state to 0 after the processing
    occupancy_integrator_->updateFromTsdfLayer(
        clear_updated_flag_tsdf, in_batch);
  }
}

// ADD(py):
void VoxbloxServer::evalEsdfEvent() {
  if (esdf_ready_) {
    updateOccFromTsdf();
    evalEsdfRefOcc();
    // float voxel_size = occupancy_map_->getOccupancyLayer().voxel_size();
    visualizeEsdfError();
  }
}

// ADD(py):
// Evaluate the accuracy of ESDF mapping, referenced to current occupancy map
// add it later to a seperate class
void VoxbloxServer::evalEsdfRefOcc() {
  timing::Timer eval_esdf_timer("eval/esdf");

  pcl::PointCloud<pcl::PointXYZ>::Ptr occ_ptcloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  BlockIndexList occ_blocks;
  occupancy_map_->getOccupancyLayer().getAllAllocatedBlocks(&occ_blocks);
  int voxels_per_side = occupancy_map_->getOccupancyLayer().voxels_per_side();
  float voxel_size = occupancy_map_->getOccupancyLayer().voxel_size();
  float error_trunc_limit = voxel_size * 2.0;

  for (const BlockIndex& block_index : occ_blocks) {
    Block<OccupancyVoxel>::ConstPtr occ_block =
        occupancy_map_->getOccupancyLayer().getBlockPtrByIndex(block_index);
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

      pcl::PointXYZ pt(point(0), point(1), point(2));
      occ_ptcloud->points.push_back(pt);
    }
  }
  // build the kd tree of the occupied grid point cloud
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::cout << "Begin to evaluate ESDF mapping accuracy ";
  kdtree.setInputCloud(occ_ptcloud);

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  BlockIndexList esdf_blocks;
  esdf_map_->getEsdfLayer().getAllAllocatedBlocks(&esdf_blocks);

  double mse = 0.0, mae = 0.0;
  uint64_t total_evaluated_voxels = 0;

  for (const BlockIndex& block_index : esdf_blocks) {
    Block<EsdfVoxel>::ConstPtr esdf_block =
        esdf_map_->getEsdfLayer().getBlockPtrByIndex(block_index);
    if (!esdf_block) {
      continue;
    }

    const size_t num_voxels_per_block = esdf_block->num_voxels();
    for (size_t lin_index = 0u; lin_index < num_voxels_per_block; ++lin_index) {
      const EsdfVoxel& esdf_voxel =
          esdf_block->getVoxelByLinearIndex(lin_index);
      if (!esdf_voxel.observed)
        continue;

      VoxelIndex voxel_index =
          esdf_block->computeVoxelIndexFromLinearIndex(lin_index);
      GlobalIndex global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(
          block_index, voxel_index, voxels_per_side);

      Point point = getCenterPointFromGridIndex(global_index, voxel_size);

      kdtree.nearestKSearch(
          pcl::PointXYZ(point(0), point(1), point(2)), 1, pointIdxNKNSearch,
          pointNKNSquaredDistance);
      float cur_gt_dist = std::sqrt(pointNKNSquaredDistance[0]);
      float cur_est_dist = std::abs(esdf_voxel.distance);
      float cur_error_dist = cur_est_dist - cur_gt_dist;
      cur_error_dist = std::min(
          error_trunc_limit, std::max(-error_trunc_limit, cur_error_dist));
      mse += (cur_error_dist * cur_error_dist);
      mae += std::abs(cur_error_dist);

      // Clamped with the error limit for visualization
      // esdf_integrator_->assignError(
      //     global_index, std::max(-error_vis_limit,
      //                            std::min(error_vis_limit, cur_error_dist)));

      esdf_integrator_->assignError(global_index, cur_error_dist);

      total_evaluated_voxels++;
    }
  }

  double rms = sqrt(mse / total_evaluated_voxels);
  mae /= total_evaluated_voxels;

  std::cout << "Finished evaluating ESDF map.\n"
            << "\nRMSE:           " << rms << "\nMAE:            " << mae
            << "\nTotal evaluated:     " << total_evaluated_voxels << "\n";

  eval_esdf_timer.Stop();

  occ_ptcloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

// ADD(py):
void VoxbloxServer::visualizeEsdfError() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;

  constexpr int kZAxisIndex = 2;
  createErrorPointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    msg.header.frame_id = world_frame_;
    msg.header.stamp = node_->now();
    esdf_error_slice_pub_->publish(msg);
  }
}

}  // namespace voxblox
