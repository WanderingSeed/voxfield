#include "voxblox_ros/voxedt_server.h"
#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

#include <pcl/kdtree/kdtree_flann.h>

namespace voxblox {

VoxedtServer::VoxedtServer(const rclcpp::Node::SharedPtr& node)
    : VoxedtServer(
          node, getEsdfMapConfigFromOccMapRosParam(node),
          getEsdfEdtIntegratorConfigFromRosParam(node),
          getTsdfMapConfigFromOccMapRosParam(node),
          getTsdfIntegratorConfigFromRosParam(node),
          getOccupancyMapConfigFromRosParam(node),
          getOccTsdfIntegratorConfigFromRosParam(node),
          getMeshIntegratorConfigFromRosParam(node)) {}

VoxedtServer::VoxedtServer(
    const rclcpp::Node::SharedPtr& node,
    const EsdfMap::Config& esdf_config,
    const EsdfOccEdtIntegrator::Config& esdf_integrator_config,
    const TsdfMap::Config& tsdf_config,
    const TsdfIntegratorBase::Config& tsdf_integrator_config,
    const OccupancyMap::Config& occ_config,
    const OccTsdfIntegrator::Config& occ_tsdf_integrator_config,
    const MeshIntegratorConfig& mesh_config)
    : TsdfServer(node, tsdf_config, tsdf_integrator_config, mesh_config),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true),
      num_subscribers_esdf_map_(0) {
  // Set up Occupancy map and integrator
  occupancy_map_.reset(new OccupancyMap(occ_config));
  occupancy_integrator_.reset(new OccTsdfIntegrator(
      occ_tsdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
      occupancy_map_->getOccupancyLayerPtr()));

  // Set up ESDF map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfOccEdtIntegrator(
      esdf_integrator_config, occupancy_map_->getOccupancyLayerPtr(),
      esdf_map_->getEsdfLayerPtr()));

  // ROS 2 specific initializations
  esdf_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "esdf_pointcloud", rclcpp::QoS(1).transient_local());
  esdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "esdf_slice", rclcpp::QoS(1).transient_local());
  traversable_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "traversable", rclcpp::QoS(1).transient_local());
  esdf_error_slice_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "esdf_error_slice", rclcpp::QoS(1).transient_local());
  esdf_map_pub_ = node_->create_publisher<voxblox_msgs::msg::Layer>(
      "esdf_map_out", rclcpp::QoS(1));

  esdf_map_sub_ = node_->create_subscription<voxblox_msgs::msg::Layer>(
      "esdf_map_in", rclcpp::QoS(1),
      std::bind(&VoxedtServer::esdfMapCallback, this, std::placeholders::_1));

  node_->get_parameter_or("clear_sphere_for_planning",
                          clear_sphere_for_planning_, clear_sphere_for_planning_);
  node_->get_parameter_or("publish_esdf_map", publish_esdf_map_,
                          publish_esdf_map_);
  node_->get_parameter_or("publish_traversable", publish_traversable_,
                          publish_traversable_);
  node_->get_parameter_or("traversability_radius", traversability_radius_,
                          traversability_radius_);

  double update_esdf_every_n_sec = 1.0;
  node_->get_parameter_or("update_esdf_every_n_sec", update_esdf_every_n_sec,
                          update_esdf_every_n_sec);

  bool eval_esdf_on = false;
  node_->get_parameter_or("eval_esdf_on", eval_esdf_on, eval_esdf_on);

  double eval_esdf_every_n_sec = 100.0;
  node_->get_parameter_or("eval_esdf_every_n_sec", eval_esdf_every_n_sec,
                          eval_esdf_every_n_sec);

  save_esdf_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "save_esdf_map",
      std::bind(&VoxedtServer::saveEsdfMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  save_occ_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "save_occ_map",
      std::bind(&VoxedtServer::saveOccMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  save_all_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "save_all_map",
      std::bind(&VoxedtServer::saveAllMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));

  if (update_esdf_every_n_sec > 0.0) {
    update_esdf_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(update_esdf_every_n_sec),
        std::bind(&VoxedtServer::updateEsdfEvent, this));
  } else {
    update_esdf_every_n_ = static_cast<int>(-1.0 * update_esdf_every_n_sec);
  }

  esdf_ready_ = false;

  if (eval_esdf_every_n_sec > 0.0 && eval_esdf_on) {
    eval_esdf_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(eval_esdf_every_n_sec),
        std::bind(&VoxedtServer::evalEsdfEvent, this));
  }
}



void VoxedtServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    esdf_pointcloud_pub_->publish(msg);
  }
}

void VoxedtServer::publishSlices() {
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

void VoxedtServer::visualizeEsdfError() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;

  constexpr int kZAxisIndex = 2;
  createErrorPointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    esdf_error_slice_pub_->publish(msg);
  }
}

// bool VoxedtServer::generateEsdfCallback(
//     std_srvs::srv::Empty::Request& /*request*/,      // NOLINT
//     std_srvs::srv::Empty::Response& /*response*/) {  // NOLINT
//   const bool clear_esdf = true;
//   if (clear_esdf) {
//     esdf_integrator_->updateFromTsdfLayerBatch();
//   } else {
//     const bool clear_updated_flag = true;
//     esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
//   }
//   publishAllUpdatedEsdfVoxels();
//   publishSlices();
//   return true;
// }

void VoxedtServer::saveAllMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response) {
  response->success = saveAllMap(request->file_path);
}

void VoxedtServer::saveEsdfMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response) {
  response->success = saveEsdfMap(request->file_path);
}

void VoxedtServer::saveOccMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response) {
  response->success = saveOccMap(request->file_path);
}

void VoxedtServer::updateEsdfEvent() {
  updateOccFromTsdf();
  updateEsdfFromOcc();
  publishOccupancyOccupiedNodes();
  // publishPointclouds();
  if (publish_slices_)
    publishSlices();
}

void VoxedtServer::evalEsdfEvent() {
  if (esdf_ready_) {
    evalEsdfRefOcc();
    visualizeEsdfError();
  }
}

void VoxedtServer::publishOccupancyOccupiedNodes() {
  visualization_msgs::msg::MarkerArray marker_array;
  createOccupancyBlocksFromOccupancyLayer(
      occupancy_map_->getOccupancyLayer(), world_frame_, &marker_array);
  occupancy_marker_pub_->publish(marker_array);
}

void VoxedtServer::publishPointclouds() {
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }
  // TsdfServer::publishPointclouds();
}

void VoxedtServer::publishTraversable() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(
      esdf_map_->getEsdfLayer(), traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pointcloud, msg);
    msg.header.frame_id = world_frame_;
    msg.header.stamp = node_->now();
    traversable_pub_->publish(msg);
  }
}

void VoxedtServer::publishMap(bool reset_remote_map) {
  if (!publish_esdf_map_) {
    return;
  }

  int subscribers = esdf_map_pub_->get_subscription_count();
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
  TsdfServer::publishMap(reset_remote_map);
}

bool VoxedtServer::saveAllMap(const std::string& file_path) {
  std::string file_path_tsdf = file_path + ".tsdf";
  std::string file_path_esdf = file_path + ".esdf";
  std::string file_path_occ = file_path + ".occ";

  return saveTsdfMap(file_path_tsdf) && saveEsdfMap(file_path_esdf) &&
         saveOccMap(file_path_occ);
}

bool VoxedtServer::saveTsdfMap(const std::string& file_path) {
  return TsdfServer::saveMap(file_path);
}

bool VoxedtServer::saveEsdfMap(const std::string& file_path) {
  constexpr bool kClearFile = false;
  return io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool VoxedtServer::saveOccMap(const std::string& file_path) {
  constexpr bool kClearFile = false;
  return io::SaveLayer(
      occupancy_map_->getOccupancyLayer(), file_path, kClearFile);
}

bool VoxedtServer::loadMap(const std::string& file_path) {
  // Load in the same order: TSDF first, then ESDF.
  bool success = TsdfServer::loadMap(file_path);

  constexpr bool kMultipleLayerSupport = true;
  return success &&
         io::LoadBlocksFromFile(
             file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
             kMultipleLayerSupport, esdf_map_->getEsdfLayerPtr());
}

// Incrementally update Esdf from occupancy map via FIESTA
void VoxedtServer::updateEsdfFromOcc() {
  if (occupancy_map_->getOccupancyLayer().getNumberOfAllocatedBlocks() > 0) {
    GlobalIndexList insert_list = occupancy_integrator_->getInsertList();
    GlobalIndexList delete_list = occupancy_integrator_->getDeleteList();
    occupancy_integrator_->clearList();
    esdf_integrator_->loadInsertList(insert_list);
    esdf_integrator_->loadDeleteList(delete_list);
    if (insert_list.size() + delete_list.size() > 0) {
      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Insert [%zu] and delete [%zu] occupied voxels.",
                    insert_list.size(), delete_list.size());

      const bool clear_updated_flag_esdf = true;
      auto start = node_->get_clock()->now();

      // set update state to 0 after the processing
      esdf_integrator_->updateFromOccLayer(clear_updated_flag_esdf);

      auto end = node_->get_clock()->now();

      if (verbose_) {
        RCLCPP_INFO(
            node_->get_logger(),
            "Finished ESDF integrating in [%f] seconds, have [%lu] "
            "blocks(memory : %f MB).",
            (end - start).seconds(),
            esdf_map_->getEsdfLayer().getNumberOfAllocatedBlocks(),
            esdf_map_->getEsdfLayer().getMemorySize() / 1024.0 /
                1024.0);  // NOLINT
      }
      esdf_ready_ = true;
    }
    esdf_integrator_->clear();
  }
}

// TODO(py): Processing in batch
// void VoxedtServer::updateEsdfBatch(bool full_euclidean) {
//   if (occupancy_map_->getOccupancyLayer().getNumberOfAllocatedBlocks() > 0) {
//     esdf_integrator_->setFullEuclidean(full_euclidean);
//     esdf_integrator_->updateFromTsdfLayerBatch();
//   }
// }

// incrementally update occupancy map from the updated TSDF map
void VoxedtServer::updateOccFromTsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    const bool in_batch = false;

    // set update state to 0 after the processing
    occupancy_integrator_->updateFromTsdfLayer(
        clear_updated_flag_esdf, in_batch);
  }
}

// Evaluate the accuracy of ESDF mapping, referenced to current occupancy map
void VoxedtServer::evalEsdfRefOcc() {
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
  RCLCPP_INFO(node_->get_logger(), "Begin to evaluate ESDF mapping accuracy ");
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
          error_trunc_limit,
          std::max(-error_trunc_limit, cur_error_dist));  // NOLINT
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

  RCLCPP_INFO(node_->get_logger(), "Finished evaluating ESDF map.\n"
            "\nRMSE:           %f\nMAE:            %f"
            "\nTotal evaluated:     %lu\n", rms, mae, total_evaluated_voxels);

  eval_esdf_timer.Stop();
  occ_ptcloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

float VoxedtServer::getEsdfMaxDistance() const {
  return esdf_integrator_->getEsdfMaxDistance();
}

void VoxedtServer::setEsdfMaxDistance(float max_distance) {
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float VoxedtServer::getTraversabilityRadius() const {
  return traversability_radius_;
}

void VoxedtServer::setTraversabilityRadius(float traversability_radius) {
  traversability_radius_ = traversability_radius;
}

void VoxedtServer::newPoseCallback(const Transformation& T_G_C) {
  // if update_esdf_every_n_sec_ is negative
  // we regard it as the update interval
  if (update_esdf_every_n_ > 0 && frame_count_ != 0 &&
      frame_count_ % update_esdf_every_n_ == 0) {
    updateOccFromTsdf();
    updateEsdfFromOcc();
    publishOccupancyOccupiedNodes();
    if (publish_slices_)
      publishSlices();
  }

  // if (clear_sphere_for_planning_) {
  //   esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
  // }

  // timing::Timer block_remove_timer("remove_distant_blocks");
  esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  // block_remove_timer.Stop();
}

//   timing::Timer block_remove_timer("remove_distant_blocks");
//   esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
//       T_G_C.getPosition(), max_block_distance_from_body_);
//   block_remove_timer.Stop();
// }

void VoxedtServer::esdfMapCallback(
    const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Got an invalid ESDF map message!");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got an ESDF map from ROS topic!");
    if (publish_pointclouds_) {
      publishPointclouds();
    }
  }
}

void VoxedtServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

}  // namespace voxblox
