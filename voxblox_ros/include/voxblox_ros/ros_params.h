#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <rclcpp/node.hpp>
#include <voxblox/core/common.h>
#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/esdf_occ_edt_integrator.h>
#include <voxblox/integrator/esdf_occ_fiesta_integrator.h>
#include <voxblox/integrator/esdf_voxfield_integrator.h>
#include <voxblox/integrator/np_tsdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/occupancy_tsdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {

namespace internal {
// Helper function to declare and get parameter in one call
template <typename T>
void getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, T* value) {
  *value = node->declare_parameter(name, *value);
}
}  // namespace internal

inline TsdfMap::Config getTsdfMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  TsdfMap::Config tsdf_config;

  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  internal::getParam(node, "tsdf_voxel_size", &voxel_size);
  internal::getParam(node, "tsdf_voxels_per_side", &voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(node->get_logger(), "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline ICP::Config getICPConfigFromRosParam(const rclcpp::Node::SharedPtr& node) {
  ICP::Config icp_config;

  internal::getParam(node, "icp_min_match_ratio", &icp_config.min_match_ratio);
  internal::getParam(node, "icp_subsample_keep_ratio", &icp_config.subsample_keep_ratio);
  internal::getParam(node, "icp_mini_batch_size", &icp_config.mini_batch_size);
  internal::getParam(node, "icp_refine_roll_pitch", &icp_config.refine_roll_pitch);
  internal::getParam(node, "icp_inital_translation_weighting", &icp_config.inital_translation_weighting);
  internal::getParam(node, "icp_inital_rotation_weighting", &icp_config.inital_rotation_weighting);

  return icp_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  TsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);

  double max_weight = integrator_config.max_weight;
  float truncation_distance = -2.0;

  internal::getParam(node, "truncation_distance", &truncation_distance);

  integrator_config.default_truncation_distance =
      truncation_distance > 0
          ? truncation_distance
          : -truncation_distance * tsdf_config.tsdf_voxel_size;

  internal::getParam(node, "voxel_carving_enabled", &integrator_config.voxel_carving_enabled);
  internal::getParam(node, "max_ray_length_m", &integrator_config.max_ray_length_m);
  internal::getParam(node, "min_ray_length_m", &integrator_config.min_ray_length_m);
  internal::getParam(node, "max_weight", &max_weight);
  integrator_config.max_weight = static_cast<float>(max_weight);
  internal::getParam(node, "use_const_weight", &integrator_config.use_const_weight);
  internal::getParam(node, "use_weight_dropoff", &integrator_config.use_weight_dropoff);
  internal::getParam(node, "allow_clear", &integrator_config.allow_clear);
  internal::getParam(node, "start_voxel_subsampling_factor", &integrator_config.start_voxel_subsampling_factor);
  internal::getParam(node, "max_consecutive_ray_collisions", &integrator_config.max_consecutive_ray_collisions);
  internal::getParam(node, "clear_checks_every_n_frames", &integrator_config.clear_checks_every_n_frames);
  internal::getParam(node, "max_integration_time_s", &integrator_config.max_integration_time_s);
  internal::getParam(node, "anti_grazing", &integrator_config.enable_anti_grazing);
  internal::getParam(node, "use_sparsity_compensation_factor", &integrator_config.use_sparsity_compensation_factor);
  internal::getParam(node, "sparsity_compensation_factor", &integrator_config.sparsity_compensation_factor);
  internal::getParam(node, "integration_order_mode", &integrator_config.integration_order_mode);
  float integrator_threads = std::thread::hardware_concurrency();
  internal::getParam(node, "integrator_threads", &integrator_threads);
  integrator_config.integrator_threads = static_cast<int>(integrator_threads);
  internal::getParam(node, "merge_with_clear", &integrator_config.merge_with_clear);

  return integrator_config;
}

inline NpTsdfIntegratorBase::Config getNpTsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  NpTsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);

  double max_weight = integrator_config.max_weight;
  float truncation_distance = -2.0;

  internal::getParam(node, "truncation_distance", &truncation_distance);

  integrator_config.default_truncation_distance =
      truncation_distance > 0
          ? truncation_distance
          : -truncation_distance * tsdf_config.tsdf_voxel_size;

  internal::getParam(node, "voxel_carving_enabled", &integrator_config.voxel_carving_enabled);
  internal::getParam(node, "max_ray_length_m", &integrator_config.max_ray_length_m);
  internal::getParam(node, "min_ray_length_m", &integrator_config.min_ray_length_m);
  internal::getParam(node, "max_weight", &max_weight);
  integrator_config.max_weight = static_cast<float>(max_weight);
  internal::getParam(node, "use_const_weight", &integrator_config.use_const_weight);
  internal::getParam(node, "use_weight_dropoff", &integrator_config.use_weight_dropoff);
  internal::getParam(node, "weight_reduction_exp", &integrator_config.weight_reduction_exp);
  internal::getParam(node, "weight_dropoff_epsilon", &integrator_config.weight_dropoff_epsilon);
  internal::getParam(node, "allow_clear", &integrator_config.allow_clear);
  internal::getParam(node, "start_voxel_subsampling_factor", &integrator_config.start_voxel_subsampling_factor);
  internal::getParam(node, "max_consecutive_ray_collisions", &integrator_config.max_consecutive_ray_collisions);
  internal::getParam(node, "clear_checks_every_n_frames", &integrator_config.clear_checks_every_n_frames);
  internal::getParam(node, "max_integration_time_s", &integrator_config.max_integration_time_s);
  internal::getParam(node, "anti_grazing", &integrator_config.enable_anti_grazing);
  internal::getParam(node, "use_sparsity_compensation_factor", &integrator_config.use_sparsity_compensation_factor);
  internal::getParam(node, "sparsity_compensation_factor", &integrator_config.sparsity_compensation_factor);
  internal::getParam(node, "integration_order_mode", &integrator_config.integration_order_mode);
  float integrator_threads = std::thread::hardware_concurrency();
  internal::getParam(node, "integrator_threads", &integrator_threads);
  integrator_config.integrator_threads = static_cast<int>(integrator_threads);
  internal::getParam(node, "merge_with_clear", &integrator_config.merge_with_clear);
  internal::getParam(node, "normal_available", &integrator_config.normal_available);
  internal::getParam(node, "reliable_band_ratio", &integrator_config.reliable_band_ratio);
  internal::getParam(node, "curve_assumption", &integrator_config.curve_assumption);
  internal::getParam(node, "reliable_normal_ratio_thre", &integrator_config.reliable_normal_ratio_thre);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfMap::Config esdf_config;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;

  return esdf_config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfIntegrator::Config esdf_integrator_config;

  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam(node);

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance / 2.0;

  internal::getParam(node, "esdf_euclidean_distance", &esdf_integrator_config.full_euclidean_distance);
  internal::getParam(node, "esdf_max_distance_m", &esdf_integrator_config.max_distance_m);
  internal::getParam(node, "esdf_min_distance_m", &esdf_integrator_config.min_distance_m);
  internal::getParam(node, "esdf_default_distance_m", &esdf_integrator_config.default_distance_m);
  internal::getParam(node, "esdf_min_diff_m", &esdf_integrator_config.min_diff_m);
  internal::getParam(node, "clear_sphere_radius", &esdf_integrator_config.clear_sphere_radius);
  internal::getParam(node, "occupied_sphere_radius", &esdf_integrator_config.occupied_sphere_radius);
  internal::getParam(node, "esdf_add_occupied_crust", &esdf_integrator_config.add_occupied_crust);

  if (esdf_integrator_config.default_distance_m < esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m = esdf_integrator_config.max_distance_m;
  }

  return esdf_integrator_config;
}

inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  MeshIntegratorConfig mesh_integrator_config;

  internal::getParam(node, "mesh_min_weight", &mesh_integrator_config.min_weight);
  internal::getParam(node, "mesh_use_color", &mesh_integrator_config.use_color);

  return mesh_integrator_config;
}

inline OccupancyMap::Config getOccupancyMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  OccupancyMap::Config occ_config;

  double voxel_size = occ_config.occupancy_voxel_size;
  int voxels_per_side = occ_config.occupancy_voxels_per_side;
  internal::getParam(node, "occ_voxel_size", &voxel_size);
  internal::getParam(node, "occ_voxels_per_side", &voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(node->get_logger(), "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = occ_config.occupancy_voxels_per_side;
  }

  occ_config.occupancy_voxel_size = static_cast<FloatingPoint>(voxel_size);
  occ_config.occupancy_voxels_per_side = voxels_per_side;

  return occ_config;
}

inline OccTsdfIntegrator::Config getOccTsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  OccTsdfIntegrator::Config integrator_config;

  internal::getParam(node, "occ_min_weight", &integrator_config.min_weight);
  internal::getParam(node, "occ_voxel_size_ratio", &integrator_config.occ_voxel_size_ratio);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromOccMapRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfMap::Config esdf_config;

  const OccupancyMap::Config occ_config = getOccupancyMapConfigFromRosParam(node);
  esdf_config.esdf_voxel_size = occ_config.occupancy_voxel_size;
  esdf_config.esdf_voxels_per_side = occ_config.occupancy_voxels_per_side;

  return esdf_config;
}

inline TsdfMap::Config getTsdfMapConfigFromOccMapRosParam(
    const rclcpp::Node::SharedPtr& node) {
  TsdfMap::Config tsdf_config;

  const OccupancyMap::Config occ_config = getOccupancyMapConfigFromRosParam(node);
  tsdf_config.tsdf_voxel_size = occ_config.occupancy_voxel_size;
  tsdf_config.tsdf_voxels_per_side = occ_config.occupancy_voxels_per_side;

  return tsdf_config;
}

inline TsdfMap::Config getTsdfMapConfigFromEsdfMapRosParam(
    const rclcpp::Node::SharedPtr& node) {
  TsdfMap::Config tsdf_config;

  const EsdfMap::Config esdf_config = getEsdfMapConfigFromRosParam(node);
  tsdf_config.tsdf_voxel_size = esdf_config.esdf_voxel_size;
  tsdf_config.tsdf_voxels_per_side = esdf_config.esdf_voxels_per_side;

  return tsdf_config;
}

inline EsdfVoxfieldIntegrator::Config getEsdfVoxfieldIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfVoxfieldIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  internal::getParam(node, "local_range_offset_x", &range_boundary_offset_x);
  internal::getParam(node, "local_range_offset_y", &range_boundary_offset_y);
  internal::getParam(node, "local_range_offset_z", &range_boundary_offset_z);
  internal::getParam(node, "esdf_max_distance_m", &esdf_integrator_config.max_distance_m);
  internal::getParam(node, "esdf_default_distance_m", &esdf_integrator_config.default_distance_m);
  internal::getParam(node, "fix_band_distance_m", &esdf_integrator_config.band_distance_m);
  internal::getParam(node, "max_behind_surface_m", &esdf_integrator_config.max_behind_surface_m);
  internal::getParam(node, "occ_min_weight", &esdf_integrator_config.min_weight);
  internal::getParam(node, "occ_voxel_size_ratio", &esdf_integrator_config.occ_voxel_size_ratio);
  internal::getParam(node, "num_buckets", &esdf_integrator_config.num_buckets);
  internal::getParam(node, "patch_on", &esdf_integrator_config.patch_on);
  internal::getParam(node, "early_break", &esdf_integrator_config.early_break);
  internal::getParam(node, "finer_esdf_on", &esdf_integrator_config.finer_esdf_on);

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

inline EsdfOccFiestaIntegrator::Config getEsdfOccFiestaIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfOccFiestaIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  internal::getParam(node, "local_range_offset_x", &range_boundary_offset_x);
  internal::getParam(node, "local_range_offset_y", &range_boundary_offset_y);
  internal::getParam(node, "local_range_offset_z", &range_boundary_offset_z);
  internal::getParam(node, "esdf_max_distance_m", &esdf_integrator_config.max_distance_m);
  internal::getParam(node, "esdf_default_distance_m", &esdf_integrator_config.default_distance_m);
  internal::getParam(node, "max_behind_surface_m", &esdf_integrator_config.max_behind_surface_m);
  internal::getParam(node, "num_buckets", &esdf_integrator_config.num_buckets);
  internal::getParam(node, "patch_on", &esdf_integrator_config.patch_on);
  internal::getParam(node, "early_break", &esdf_integrator_config.early_break);

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

inline EsdfOccEdtIntegrator::Config getEsdfEdtIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfOccEdtIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  internal::getParam(node, "local_range_offset_x", &range_boundary_offset_x);
  internal::getParam(node, "local_range_offset_y", &range_boundary_offset_y);
  internal::getParam(node, "local_range_offset_z", &range_boundary_offset_z);
  internal::getParam(node, "esdf_max_distance_m", &esdf_integrator_config.max_distance_m);
  internal::getParam(node, "esdf_default_distance_m", &esdf_integrator_config.default_distance_m);
  internal::getParam(node, "max_behind_surface_m", &esdf_integrator_config.max_behind_surface_m);
  internal::getParam(node, "num_buckets", &esdf_integrator_config.num_buckets);
 if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_