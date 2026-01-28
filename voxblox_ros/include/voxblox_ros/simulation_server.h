#ifndef VOXBLOX_ROS_SIMULATION_SERVER_H_
#define VOXBLOX_ROS_SIMULATION_SERVER_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voxblox_msgs/msg/layer.hpp>
#include <voxblox_msgs/msg/mesh.hpp>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/esdf_occ_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/simulation/simulation_world.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

class SimulationServer {
 public:
    // ROS2 constructor
    SimulationServer(const rclcpp::Node::SharedPtr& node);

  virtual ~SimulationServer() {}

  /// Runs all of the below functions in the correct order:
  void run();

  /// Creates a new world, and prepares ground truth SDF(s).
  virtual void prepareWorld() = 0;

  /// Generates a SDF by generating random poses and putting them into an SDF.
  void generateSDF();

  /// Evaluate errors...
  void evaluate();

  /// Visualize results. :)
  void visualize();

 protected:

  /// Convenience function to generate valid viewpoints.
  bool generatePlausibleViewpoint(
      FloatingPoint min_distance, Point* ray_origin,
      Point* ray_direction) const;

  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr sim_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_gt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_gt_pub_;
  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr tsdf_gt_mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_test_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_test_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tsdf_test_mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr view_ptcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sdf_slice_pub_;

  // Settings
  FloatingPoint voxel_size_;
  int voxels_per_side_;
  std::string world_frame_;
  bool generate_occupancy_;
  bool visualize_;
  FloatingPoint visualization_slice_level_;
  bool generate_mesh_;
  bool incremental_;
  bool add_robot_pose_;
  FloatingPoint truncation_distance_;
  FloatingPoint esdf_max_distance_;
  size_t max_attempts_to_generate_viewpoint_;

  // Camera settings
  Eigen::Vector2i depth_camera_resolution_;
  FloatingPoint fov_h_rad_;
  FloatingPoint max_dist_;
  FloatingPoint min_dist_;
  int num_viewpoints_;

  // Actual simulation server.
  std::unique_ptr<SimulationWorld> world_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_gt_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_gt_;

  // Generated maps:
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_test_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_test_;
  std::unique_ptr<Layer<OccupancyVoxel> > occ_test_;

  // ROS2 node pointer (optional, used during staged migration).
  rclcpp::Node::SharedPtr node_;

  // Integrators.
  std::shared_ptr<TsdfIntegratorBase> tsdf_integrator_;
  std::shared_ptr<OccupancyIntegrator> occ_integrator_;
  std::shared_ptr<EsdfIntegrator> esdf_integrator_;
  std::shared_ptr<EsdfOccIntegrator> esdf_occ_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_SIMULATION_SERVER_H_
