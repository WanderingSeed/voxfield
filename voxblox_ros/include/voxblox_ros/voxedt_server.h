#ifndef VOXBLOX_ROS_VOXEDT_SERVER_H_
#define VOXBLOX_ROS_VOXEDT_SERVER_H_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/integrator/esdf_occ_edt_integrator.h>
#include <voxblox/integrator/esdf_occ_fiesta_integrator.h>
#include <voxblox/integrator/occupancy_tsdf_integrator.h>
#include <voxblox_msgs/msg/layer.hpp>

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

class VoxedtServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // ROS2 constructor
  VoxedtServer(const rclcpp::Node::SharedPtr& node);
  VoxedtServer(
      const rclcpp::Node::SharedPtr& nh,
      const EsdfMap::Config& esdf_config,
      const EsdfOccEdtIntegrator::Config& esdf_integrator_config,
      const TsdfMap::Config& tsdf_config,
      const TsdfIntegratorBase::Config& tsdf_integrator_config,
      const OccupancyMap::Config& occ_config,
      const OccTsdfIntegrator::Config& occ_tsdf_integrator_config,
      const MeshIntegratorConfig& mesh_config);
  virtual ~VoxedtServer() {}

  void publishAllUpdatedEsdfVoxels();
  virtual void publishSlices();
  void visualizeEsdfError();
  void publishTraversable();
  void publishOccupancyOccupiedNodes();

  virtual void publishPointclouds();
  virtual void newPoseCallback(const Transformation& T_G_C);
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveTsdfMap(const std::string& file_path);
  virtual bool saveEsdfMap(const std::string& file_path);
  virtual bool saveOccMap(const std::string& file_path);
  virtual bool saveAllMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  /// Timer events
  void updateEsdfEvent();

  void evalEsdfEvent();

  /// Call this to update the ESDF based on latest state of the occupancy map,
  /// considering only the newly updated parts of the occupancy map (checked
  /// with the ESDF updated bit in Update::Status).
  void updateEsdfFromOcc();

  /// Update the ESDF all at once; clear the existing map.
  // void updateEsdfBatch(bool full_euclidean = false);

  /// Call this to update the Occupancy map based on latest state of the TSDF
  /// map
  void updateOccFromTsdf();

  void evalEsdfRefOcc();

  // Overwrites the layer with what's coming from the topic!
  void esdfMapCallback(const std::shared_ptr<voxblox_msgs::msg::Layer> layer_msg);

  inline std::shared_ptr<EsdfMap> getEsdfMapPtr() {
    return esdf_map_;
  }
  inline std::shared_ptr<const EsdfMap> getEsdfMapPtr() const {
    return esdf_map_;
  }

  void saveEsdfMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,     // NOLINT
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response);  // NOLINT

  void saveOccMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,     // NOLINT
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response);  // NOLINT

  void saveAllMapCallback(
    const std::shared_ptr<voxblox_msgs::srv::FilePath::Request> request,     // NOLINT
    std::shared_ptr<voxblox_msgs::srv::FilePath::Response> response);  // NOLINT

  bool getClearSphere() const {
    return clear_sphere_for_planning_;
  }
  
  void setClearSphere(bool clear_sphere_for_planning) {
    clear_sphere_for_planning_ = clear_sphere_for_planning;
  }

  float getEsdfMaxDistance() const;
  void setEsdfMaxDistance(float max_distance);

  float getTraversabilityRadius() const;
  void setTraversabilityRadius(float traversability_radius);

  /**
   * These are for enabling or disabling incremental update of the ESDF. Use
   * carefully.
   */
  void disableIncrementalUpdate() {
    incremental_update_ = false;
  }
  void enableIncrementalUpdate() {
    incremental_update_ = true;
  }

  virtual void clear();

 protected:
  /// Sets up publishing and subscribing. Should only be called from
  /// constructor.
  void setupRos();

  rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr esdf_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_slice_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_error_slice_pub_;

  rclcpp::Subscription<voxblox_msgs::msg::Layer>::SharedPtr esdf_map_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_esdf_srv_;
  rclcpp::Service<voxblox_msgs::srv::FilePath>::SharedPtr save_esdf_map_srv_;
  rclcpp::Service<voxblox_msgs::srv::FilePath>::SharedPtr save_occ_map_srv_;
  rclcpp::Service<voxblox_msgs::srv::FilePath>::SharedPtr save_all_map_srv_;

  rclcpp::TimerBase::SharedPtr update_esdf_timer_;
  rclcpp::TimerBase::SharedPtr eval_esdf_timer_;

  bool clear_sphere_for_planning_;
  bool publish_esdf_map_;
  bool publish_traversable_;
  float traversability_radius_;
  bool incremental_update_;
  int num_subscribers_esdf_map_;
  bool esdf_ready_;

  int update_esdf_every_n_ = 0;

  // ESDF maps.
  std::shared_ptr<EsdfMap> esdf_map_;
  // std::unique_ptr<EsdfOccFiestaIntegrator> esdf_integrator_;
  std::unique_ptr<EsdfOccEdtIntegrator> esdf_integrator_;

  // Occupancy maps.
  std::shared_ptr<OccupancyMap> occupancy_map_;
  std::unique_ptr<OccTsdfIntegrator> occupancy_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_VOXEDT_SERVER_H_
