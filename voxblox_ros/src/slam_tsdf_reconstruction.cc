#include "voxblox_ros/slam_tsdf_reconstruction.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

SlamTsdfReconstruction::SlamTsdfReconstruction(const rclcpp::Node::SharedPtr& node)
    : SlamTsdfReconstruction(
          node, getTsdfMapConfigFromRosParam(node),
          getNpTsdfIntegratorConfigFromRosParam(node),
          getMeshIntegratorConfigFromRosParam(node)) {}

SlamTsdfReconstruction::SlamTsdfReconstruction(
    const rclcpp::Node::SharedPtr& node, const TsdfMap::Config& config,
    const NpTsdfIntegratorBase::Config& integrator_config,
    const MeshIntegratorConfig& mesh_config)
    : node_(node),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      color_map_(new RainbowColorMap()),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0),
      transformer_(node),
      last_msg_time_ptcloud_(0, 0, RCL_CLOCK_UNINITIALIZED),
      min_time_between_msgs_(rclcpp::Duration::from_seconds(0.0))  {
  // Get general parameters.
  getServerConfigFromRosParam(node);

  // Advertise topics.
  surface_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "surface_pointcloud", rclcpp::QoS(1).transient_local());
  tsdf_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "tsdf_pointcloud", rclcpp::QoS(1).transient_local());
  gsdf_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "gsdf_pointcloud", rclcpp::QoS(1).transient_local());
  occupancy_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "occupied_nodes", rclcpp::QoS(1).transient_local());
  tsdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "tsdf_slice", rclcpp::QoS(1).transient_local());
  gsdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "gsdf_slice", rclcpp::QoS(1).transient_local());

  mesh_pub_ = node_->create_publisher<voxblox_msgs::msg::Mesh>(
      "mesh", rclcpp::QoS(1).transient_local());

  tsdf_map_pub_ = node_->create_publisher<voxblox_msgs::msg::Layer>(
      "tsdf_map_out", rclcpp::QoS(1));
  tsdf_map_sub_ = node_->create_subscription<voxblox_msgs::msg::Layer>(
      "tsdf_map_in", rclcpp::QoS(1),
      std::bind(&SlamTsdfReconstruction::tsdfMapCallback, this, std::placeholders::_1));

  if (publish_robot_model_) {
    robot_model_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            "robot_model", rclcpp::QoS(100));
  }

  if (enable_icp_) {
    icp_transform_pub_ =
        node_->create_publisher<geometry_msgs::msg::TransformStamped>(
            "icp_transform", rclcpp::QoS(1).transient_local());
    internal::getParam(node_, "icp_corrected_frame", &icp_corrected_frame_);
    internal::getParam(node_, "pose_corrected_frame", &pose_corrected_frame_);
  }

  // Initialize TSDF Map and integrator.
  tsdf_map_.reset(new TsdfMap(config));

  std::string method("merged");
  internal::getParam(node_, "method", &method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_.reset(new SimpleNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("merged") == 0) {
    tsdf_integrator_.reset(new MergedNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_.reset(new FastNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  } else {
    tsdf_integrator_.reset(new SimpleNpTsdfIntegrator(
        integrator_config, tsdf_map_->getTsdfLayerPtr()));
  }

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));
  icp_.reset(new ICP(getICPConfigFromRosParam(node_)));

  // Advertise services.
  generate_mesh_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "generate_mesh",
      std::bind(&SlamTsdfReconstruction::generateMeshCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  clear_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "clear_map",
      std::bind(&SlamTsdfReconstruction::clearMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  save_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "save_map",
      std::bind(&SlamTsdfReconstruction::saveMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  load_map_srv_ = node_->create_service<voxblox_msgs::srv::FilePath>(
      "load_map",
      std::bind(&SlamTsdfReconstruction::loadMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  publish_pointclouds_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "publish_pointclouds",
      std::bind(&SlamTsdfReconstruction::publishPointcloudsCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  publish_tsdf_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "publish_map",
      std::bind(&SlamTsdfReconstruction::publishTsdfMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0;
  internal::getParam(node_, "update_mesh_every_n_sec", &update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(update_mesh_every_n_sec),
        [this]() { this->updateMesh(); });
  } else {
    update_mesh_every_n_ = static_cast<int>(-1.0 * update_mesh_every_n_sec);
  }

  double publish_map_every_n_sec = 1.0;
  internal::getParam(node_, "publish_map_every_n_sec", &publish_map_every_n_sec);

  if (publish_map_every_n_sec > 0.0) {
    publish_map_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(publish_map_every_n_sec),
        [this]() { this->publishMap(); });
  }

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);
}

void SlamTsdfReconstruction::getServerConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  // Before subscribing, determine minimum time between messages.
  // 0 by default.
  double min_time_between_msgs_sec = 0.0;
  internal::getParam(node, "min_time_between_msgs_sec", &min_time_between_msgs_sec);
  min_time_between_msgs_ =
      rclcpp::Duration::from_seconds(min_time_between_msgs_sec);

  internal::getParam(node, "max_block_distance_from_body", &max_block_distance_from_body_);
  internal::getParam(node, "slice_level", &slice_level_);
  internal::getParam(node, "world_frame", &world_frame_);
  internal::getParam(node, "sensor_frame", &sensor_frame_);
  internal::getParam(node, "publish_pointclouds_on_update", &publish_pointclouds_on_update_);
  internal::getParam(node, "publish_slices", &publish_slices_);
  internal::getParam(node, "publish_pointclouds", &publish_pointclouds_);
  internal::getParam(node, "pointcloud_queue_size", &pointcloud_queue_size_);
  internal::getParam(node, "enable_icp", &enable_icp_);
  internal::getParam(node, "accumulate_icp_corrections", &accumulate_icp_corrections_);
  internal::getParam(node, "verbose", &verbose_);
  internal::getParam(node, "timing", &timing_);
  internal::getParam(node, "sensor_is_lidar", &sensor_is_lidar_);
  internal::getParam(node, "width", &width_);
  internal::getParam(node, "height", &height_);
  internal::getParam(node, "smooth_thre_ratio", &smooth_thre_ratio_);
  internal::getParam(node, "min_z", &min_z_);
  internal::getParam(node, "min_dist", &min_dist_);

  if (sensor_is_lidar_) {
    internal::getParam(node, "fov_up", &fov_up_);
    internal::getParam(node, "fov_down", &fov_down_);
    float fov = std::abs(fov_down_) + std::abs(fov_up_);
    fov_down_rad_ = fov_down_ / 180.0f * M_PI;
    fov_rad_ = fov / 180.0f * M_PI;
} else {
  internal::getParam(node, "vx", &vx_);
  internal::getParam(node, "vy", &vy_);
  internal::getParam(node, "fx", &fx_);
  internal::getParam(node, "fy", &fy_);
}

internal::getParam(node, "publish_robot_model", &publish_robot_model_);
internal::getParam(node, "robot_model_file", &robot_model_file_);
internal::getParam(node, "robot_model_scale", &robot_model_scale_);
internal::getParam(node, "mesh_filename", &mesh_filename_);
std::string color_mode_str = "";
internal::getParam(node, "color_mode", &color_mode_str);
color_mode_ = getColorModeFromString(color_mode_str);

std::string intensity_colormap = "rainbow";
float intensity_max_value = kDefaultMaxIntensity;
internal::getParam(node, "intensity_colormap", &intensity_colormap);
internal::getParam(node, "intensity_max_value", &intensity_max_value);

if (intensity_colormap == "rainbow") {
  color_map_.reset(new RainbowColorMap());
} else if (intensity_colormap == "inverse_rainbow") {
  color_map_.reset(new InverseRainbowColorMap());
} else if (intensity_colormap == "grayscale") {
  color_map_.reset(new GrayscaleColorMap());
} else if (intensity_colormap == "inverse_grayscale") {
  color_map_.reset(new InverseGrayscaleColorMap());
} else if (intensity_colormap == "ironbow") {
  color_map_.reset(new IronbowColorMap());
} else {
  RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Invalid color map: " << intensity_colormap);
}
color_map_->setMaxValue(intensity_max_value);
}

void SlamTsdfReconstruction::processPointCloudMessageAndInsert(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg,
    const Transformation& T_G_C) {
  timing::Timer ptcloud_timer("preprocess/input");
  // Convert the PCL pointcloud into our awesome format.
  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  bool has_label = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    } else if (pointcloud_msg->fields[d].name == std::string("label")) {
      has_label = true;
    }
  }

  Pointcloud points_C;
  Pointcloud normals_C;
  Colors colors;
  Labels labels;

  // Convert differently depending on RGB or I type.
  if (has_label) {
    pcl::PointCloud<pcl::PointXYZRGBL> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors, &labels,
                      true);
  } else if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();

  // calculate point-wise normal
  timing::Timer range_pre_timer("preprocess/normal_estimation");
  // Preprocess the point cloud: convert to range images
  cv::Mat vertex_map = cv::Mat::zeros(height_, width_, CV_32FC3);
  cv::Mat depth_image(vertex_map.size(), CV_32FC1, -1.0);
  cv::Mat color_image = cv::Mat::zeros(vertex_map.size(), CV_8UC3);
  projectPointCloudToImage(points_C, colors, vertex_map, depth_image,
                           color_image, min_z_, min_dist_);
  cv::Mat normal_image = computeNormalImage(vertex_map, depth_image);
  // Back project to point cloud from range images
  points_C = extractPointCloud(vertex_map, depth_image);
  normals_C = extractNormals(normal_image, depth_image);
  colors = extractColors(color_image, depth_image);
  
  range_pre_timer.Stop();

  // ICP based pose refinement
  Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    static Transformation T_offset;
    const size_t num_icp_updates =
        icp_->runICP(tsdf_map_->getTsdfLayer(), points_C,
                     icp_corrected_transform_ * T_G_C, &T_G_C_refined);
    if (verbose_) {
      RCLCPP_INFO(node_->get_logger(),
                  "ICP refinement performed %zu successful update steps",
                  num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      // its already removed internally but small floating point errors can
      // build up if accumulating transforms
      Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = Transformation::exp(T_vec);
    }

    geometry_msgs::msg::TransformStamped icp_tf_msg;
    geometry_msgs::msg::TransformStamped pose_tf_msg;
    geometry_msgs::msg::TransformStamped transform_msg;

    tf::transformKindrToMsg(
        icp_corrected_transform_.cast<double>(), &icp_tf_msg.transform);
    tf::transformKindrToMsg(T_G_C.cast<double>(), &pose_tf_msg.transform);
    tf::transformKindrToMsg(
        icp_corrected_transform_.cast<double>(), &transform_msg.transform);

    icp_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    icp_tf_msg.header.frame_id = world_frame_;
    icp_tf_msg.child_frame_id = icp_corrected_frame_;
    tf_broadcaster_->sendTransform(icp_tf_msg);
    pose_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    pose_tf_msg.header.frame_id = icp_corrected_frame_;
    pose_tf_msg.child_frame_id = pose_corrected_frame_;
    tf_broadcaster_->sendTransform(pose_tf_msg);
    transform_msg.header.stamp = pointcloud_msg->header.stamp;
    transform_msg.header.frame_id = world_frame_;
    transform_msg.child_frame_id = icp_corrected_frame_;
    icp_transform_pub_->publish(transform_msg);
    icp_timer.Stop();
  }

  if (verbose_) {
    RCLCPP_INFO(
        node_->get_logger(), "Integrating a pointcloud with %lu points.",
        points_C.size());
  }
  integratePointcloud(T_G_C_refined, points_C, normals_C, colors);

  // mesh reconstruction with the counter interval
  if (update_mesh_every_n_ > 0 && frame_count_ != 0 &&
      frame_count_ % update_mesh_every_n_ == 0) {
    updateMesh();
  }

  // timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(
      T_G_C.getPosition(), max_block_distance_from_body_);
  // block_remove_timer.Stop();

  publishRobotMesh(T_G_C_refined);

  // Callback for inheriting classes.
  newPoseCallback(T_G_C);
}

void SlamTsdfReconstruction::publishRobotMesh(const Transformation& T_G_C) {
  if (!publish_robot_model_) {
    return;
  }
  
  // publish the robot model with the pose
  visualization_msgs::msg::Marker robot_model;
  robot_model.header.frame_id = world_frame_;
  robot_model.header.stamp = node_->now();
  robot_model.ns = "robot";
  robot_model.id = 0;
  robot_model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  robot_model.action = visualization_msgs::msg::Marker::ADD;
  robot_model.mesh_resource = "file://" + robot_model_file_;
  robot_model.scale.x = robot_model_scale_;
  robot_model.scale.y = robot_model_scale_;
  robot_model.scale.z = robot_model_scale_;
  robot_model.color.a = 1.0;
  robot_model.color.r = 1.0;
  robot_model.color.g = 1.0;
  robot_model.color.b = 1.0;
  // Change to horizontal camera frame
  Transformation T_G_CH = T_G_C * transformer_.getModelTransform();
  Eigen::Quaternionf quatrot = T_G_CH.getEigenQuaternion();
  Point quat_vec = quatrot.vec();
  robot_model.pose.orientation.x = quat_vec(0);
  robot_model.pose.orientation.y = quat_vec(1);
  robot_model.pose.orientation.z = quat_vec(2);
  robot_model.pose.orientation.w = quatrot.w();
  Point translation = T_G_CH.getPosition();
  robot_model.pose.position.x = translation(0);
  robot_model.pose.position.y = translation(1);
  robot_model.pose.position.z = translation(2);
  robot_model_pub_->publish(robot_model);
}

// Direct interface method with voxblox Pointcloud - calls ROS message version
void SlamTsdfReconstruction::processPointcloudDirect(
    const Pointcloud& points_C, 
    const Transformation& T_G_C) {
  
  if (verbose_) {
    RCLCPP_INFO(
        node_->get_logger(), "Processing pointcloud with %lu points directly.",
        points_C.size());
  }

  // Create a ROS message for processing
  auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pointcloud_msg->header.stamp = node_->now();
  pointcloud_msg->header.frame_id = sensor_frame_;
  
  // Convert voxblox pointcloud to PCL format
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  for (const auto& point : points_C) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    pcl_cloud.points.push_back(pcl_point);
  }
  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;
  
  // Convert PCL to ROS message
  pcl::toROSMsg(pcl_cloud, *pointcloud_msg);
  
  // Call the ROS message version
  processPointcloudDirect(pointcloud_msg, T_G_C);
}


// Direct interface method with ROS PointCloud2 message - most efficient
void SlamTsdfReconstruction::processPointcloudDirect(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg,
    const Transformation& T_G_C) {

  rclcpp::Time header_stamp(pointcloud_msg->header.stamp);

   RCLCPP_INFO(
        node_->get_logger(), "new time %f last time %f.",
        header_stamp.seconds(), last_msg_time_ptcloud_.seconds());
    try
    {
      if (header_stamp - last_msg_time_ptcloud_ > min_time_between_msgs_) {
        RCLCPP_INFO(node_->get_logger(), "update time");
        last_msg_time_ptcloud_ = header_stamp;
      } else {
        return;
      }
    }
    catch (const std::runtime_error&)
    {
        // Handle exceptions when the time source changes and initialize publish timestamp
        last_msg_time_ptcloud_ = header_stamp;
    }

  
  // Call the existing processing method
  processPointCloudMessageAndInsert(pointcloud_msg, T_G_C);

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (timing_)
    RCLCPP_INFO_STREAM(node_->get_logger(), 
        "Frame [" << frame_count_ << "] timings: " << std::endl
                  << timing::Timing::Print());
  if (verbose_)
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  frame_count_++;
}

void SlamTsdfReconstruction::integratePointcloud(const Transformation& T_G_C,
                                       const Pointcloud& points_C,
                                       const Pointcloud& normals_C,
                                       const Colors& colors) {
  CHECK_EQ(points_C.size(), colors.size());
  CHECK_EQ(points_C.size(), normals_C.size());
  tsdf_integrator_->integratePointCloud(
      T_G_C, points_C, normals_C, colors, false);
}

void SlamTsdfReconstruction::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud_d;
  createDistancePointcloudFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), &pointcloud_d);
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud_d, pointcloud_msg);
  pointcloud_msg.header.frame_id = world_frame_;

  tsdf_pointcloud_pub_->publish(pointcloud_msg);
  pcl::PointCloud<pcl::PointXYZI> pointcloud_g;
  createGradientPointcloudFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), &pointcloud_g);

  pcl::toROSMsg(pointcloud_g, pointcloud_msg);
  pointcloud_msg.header.frame_id = world_frame_;
  gsdf_pointcloud_pub_->publish(pointcloud_msg);
}

void SlamTsdfReconstruction::publishTsdfSurfacePoints() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(),
                                       surface_distance_thresh, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  surface_pointcloud_pub_->publish(pointcloud_msg);
}

void SlamTsdfReconstruction::publishTsdfOccupiedNodes() {
  visualization_msgs::msg::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(tsdf_map_->getTsdfLayer(), world_frame_,
                                     &marker_array);
  occupancy_marker_pub_->publish(marker_array);
}

void SlamTsdfReconstruction::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud_d;
  createDistancePointcloudFromTsdfLayerSlice(
      tsdf_map_->getTsdfLayer(), 2, slice_level_, &pointcloud_d);

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud_d, pointcloud_msg);
  pointcloud_msg.header.frame_id = world_frame_;
  tsdf_slice_pub_->publish(pointcloud_msg);

  pcl::PointCloud<pcl::PointXYZI> pointcloud_g;
  createGradientPointcloudFromTsdfLayerSlice(
      tsdf_map_->getTsdfLayer(), 2, slice_level_, &pointcloud_g);
  pcl::toROSMsg(pointcloud_g, pointcloud_msg);
  pointcloud_msg.header.frame_id = world_frame_;
  gsdf_slice_pub_->publish(pointcloud_msg);
}
void SlamTsdfReconstruction::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  int subscribers = tsdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(
        tsdf_map_->getTsdfLayer(), only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(
          voxblox_msgs::msg::Layer::ACTION_RESET);
    }
    tsdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = subscribers;
}

void SlamTsdfReconstruction::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void SlamTsdfReconstruction::updateMesh() {
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool SlamTsdfReconstruction::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      RCLCPP_INFO(
          node_->get_logger(), "Output file as PLY: %s",
          mesh_filename_.c_str());
    } else {
      RCLCPP_INFO(
          node_->get_logger(), "Failed to output mesh as PLY: %s",
          mesh_filename_.c_str());
    }
  }

  if (timing_)
    RCLCPP_INFO(
        node_->get_logger(), "Mesh Timings: %s", timing::Timing::Print().c_str());
  return true;
}

bool SlamTsdfReconstruction::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool SlamTsdfReconstruction::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load
  // the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

void SlamTsdfReconstruction::clearMapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  clear();
}

void SlamTsdfReconstruction::generateMeshCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  generateMesh();
}

void SlamTsdfReconstruction::saveMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response) {
  response->success = saveMap(request->file_path);
}

void SlamTsdfReconstruction::loadMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response) {
  response->success = loadMap(request->file_path);
}

void SlamTsdfReconstruction::publishPointcloudsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  publishPointclouds();
}

void SlamTsdfReconstruction::publishTsdfMapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  publishMap(true);
}

void SlamTsdfReconstruction::updateMeshEvent(
    const rclcpp::TimerBase::SharedPtr /*event*/) {
  updateMesh();
}

void SlamTsdfReconstruction::publishMapEvent(
    const rclcpp::TimerBase::SharedPtr /*event*/) {
  publishMap();
}

void SlamTsdfReconstruction::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  if (publish_tsdf_map_) {
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
  }
}

void SlamTsdfReconstruction::tsdfMapCallback(
    const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      deserializeMsgToLayer<TsdfVoxel>(layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Got an invalid TSDF map message!");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got a TSDF map from a topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

bool SlamTsdfReconstruction::projectPointCloudToImage(
    const Pointcloud& points_C, const Colors& colors, cv::Mat& vertex_map,
    cv::Mat& depth_image, cv::Mat& color_image, float min_z,
    float min_d) const {
  // TODO(py): consider to calculate in parallel to speed up
  for (size_t i = 0; i < points_C.size(); i++) {
    int u, v;
    float depth;
    if (sensor_is_lidar_)
      depth = projectPointToImageLiDAR(points_C[i], &u, &v);
    else
      depth = projectPointToImageCamera(points_C[i], &u, &v);
    if (depth > min_d && points_C[i].z() > min_z) {
      float old_depth = depth_image.at<float>(v, u);
      // save only nearest point for each pixel
      if (old_depth <= 0.0 || old_depth > depth) {
        for (int k = 0; k <= 2; k++) {
          vertex_map.at<cv::Vec3f>(v, u)[k] = points_C[i](k);
        }
        depth_image.at<float>(v, u) = depth;
        // BGR default order
        color_image.at<cv::Vec3b>(v, u)[0] = colors[i].b;
        color_image.at<cv::Vec3b>(v, u)[1] = colors[i].g;
        color_image.at<cv::Vec3b>(v, u)[2] = colors[i].r;
      }
    }
  }
  return false;
}

// point should be in the LiDAR's coordinate system
float SlamTsdfReconstruction::projectPointToImageLiDAR(
    const Point& p_C, int* u, int* v) const {
  // All values are ceiled and floored to guarantee that the resulting points
  // will be valid for any integer conversion.
  float depth =
      std::sqrt(p_C.x() * p_C.x() + p_C.y() * p_C.y() + p_C.z() * p_C.z());
  float yaw = std::atan2(p_C.y(), p_C.x());
  float pitch = std::asin(p_C.z() / depth);
  // projections in image coordinates (percentage)
  float proj_x = 0.5 * (yaw / M_PI + 1.0);
  float proj_y = 1.0 - (pitch - fov_down_rad_) / fov_rad_;
  // scale to image size
  proj_x *= width_;
  proj_y *= height_;
  // round for integer index
  CHECK_NOTNULL(u);
  *u = std::round(proj_x);
  if (*u == width_)
    *u = 0;

  CHECK_NOTNULL(v);
  *v = std::round(proj_y);
  if (std::ceil(proj_y) > height_ - 1 || std::floor(proj_y) < 0) {
    return (-1.0);
  }
  return depth;
}

bool SlamTsdfReconstruction::projectPointToImageCamera(
    const Point& p_C, int* u, int* v) const {
  CHECK_NOTNULL(u);
  *u = std::round(p_C.x() * fx_ / p_C.z() + vx_);
  if (*u >= width_ || *u < 0) {
    return false;
  }
  CHECK_NOTNULL(v);
  *v = std::round(p_C.y() * fy_ / p_C.z() + vy_);
  if (*v >= height_ || *v < 0) {
    return false;
  }
  return true;
}

cv::Mat SlamTsdfReconstruction::computeNormalImage(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  cv::Mat normal_image(depth_image.size(), CV_32FC3, 0.0);
  for (int u = 0; u < width_; u++) {
    for (int v = 0; v < height_; v++) {
      Point p;
      p << vertex_map.at<cv::Vec3f>(v, u)[0], vertex_map.at<cv::Vec3f>(v, u)[1],
          vertex_map.at<cv::Vec3f>(v, u)[2];

      float d_p = depth_image.at<float>(v, u);
      // sign of the normal vector
      float sign = 1.0;

      if (d_p > 0) {
        // neighbor x (in ring)
        int n_x_u;
        if (u == width_ - 1)
          n_x_u = 0;
        else
          n_x_u = u + 1;
        Point n_x;
        n_x << vertex_map.at<cv::Vec3f>(v, n_x_u)[0],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[1],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[2];
        float d_n_x = depth_image.at<float>(v, n_x_u);
        if (d_n_x < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_x - d_p) > smooth_thre_ratio_ * d_p)
          continue;

        // neighbor y
        int n_y_v;
        if (v == height_) {
          n_y_v = v - 1;
          sign *= -1.0;
        } else {
          n_y_v = v + 1;
        }
        Point n_y;
        n_y << vertex_map.at<cv::Vec3f>(n_y_v, u)[0],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[1],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[2];

        float d_n_y = depth_image.at<float>(n_y_v, u);
        if (d_n_y < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_y - d_p) > smooth_thre_ratio_ * d_p)
          continue;
        Point dx = n_x - p;
        Point dy = n_y - p;

        Point normal = (dx.cross(dy)).normalized() * sign;
        cv::Vec3f& normals = normal_image.at<cv::Vec3f>(v, u);
        for (int k = 0; k <= 2; k++)
          normals[k] = normal(k);
      }
    }
  }
  return normal_image;
}

Pointcloud SlamTsdfReconstruction::extractPointCloud(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  Pointcloud points_C;
  for (int v = 0; v < vertex_map.rows; v++) {
    for (int u = 0; u < vertex_map.cols; u++) {
      cv::Vec3f vertex = vertex_map.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Point p_C(vertex[0], vertex[1], vertex[2]);
        points_C.push_back(p_C);
      }
    }
  }
  return points_C;
}

Colors SlamTsdfReconstruction::extractColors(
    const cv::Mat& color_image, const cv::Mat& depth_image) const {
  Colors colors;
  for (int v = 0; v < color_image.rows; v++) {
    for (int u = 0; u < color_image.cols; u++) {
      // BGR
      cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        // RGB
        Color c_C(color[2], color[1], color[0]);
        colors.push_back(c_C);
      }
    }
  }
  return colors;
}

Pointcloud SlamTsdfReconstruction::extractNormals(
    const cv::Mat& normal_image, const cv::Mat& depth_image) const {
  Pointcloud normals_C;
  for (int v = 0; v < normal_image.rows; v++) {
    for (int u = 0; u < normal_image.cols; u++) {
      cv::Vec3f vertex = normal_image.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Ray n_C(vertex[0], vertex[1], vertex[2]);
        normals_C.push_back(n_C);
      }
    }
  }
  return normals_C;
}

}  // namespace voxblox