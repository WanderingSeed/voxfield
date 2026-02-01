############
cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

find_package(glog REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(voxblox REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(minkindr REQUIRED)
find_package(minkindr_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/interactive_slider.cc
  src/simulation_server.cc
  src/intensity_server.cc
  src/transformer.cc
  src/tsdf_server.cc
  src/np_tsdf_server.cc
  src/slam_tsdf_reconstruction.cc
  src/voxfield_server.cc
  src/voxblox_server.cc
  src/voxedt_server.cc
  src/fiesta_server.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../voxblox/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../minkindr/minkindr/include>
)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  pcl_conversions
  PCL
  voxblox_msgs
  interactive_markers
  visualization_msgs
  tf2_eigen
  std_srvs
  minkindr
  minkindr_conversions
  tf2_geometry_msgs
  voxblox
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME} glog::glog)
if(DEFINED voxblox_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${voxblox_INCLUDE_DIRS})
endif()
 

############
# BINARIES #
############

add_executable(voxblox_eval src/voxblox_eval.cc)
target_link_libraries(voxblox_eval ${PROJECT_NAME})

add_executable(tsdf_server src/tsdf_server_node.cc)
target_link_libraries(tsdf_server ${PROJECT_NAME})

add_executable(np_tsdf_server src/np_tsdf_server_node.cc)
target_link_libraries(np_tsdf_server ${PROJECT_NAME})

add_executable(intensity_server src/intensity_server_node.cc)
target_link_libraries(intensity_server ${PROJECT_NAME})

add_executable(simulation_eval src/simulation_eval.cc)
target_link_libraries(simulation_eval ${PROJECT_NAME})

add_executable(visualize_tsdf src/visualize_tsdf.cc)
target_link_libraries(visualize_tsdf ${PROJECT_NAME})

add_executable(voxfield_server src/voxfield_server_node.cc)
target_link_libraries(voxfield_server ${PROJECT_NAME})

add_executable(voxblox_server src/voxblox_server_node.cc)
target_link_libraries(voxblox_server ${PROJECT_NAME})

add_executable(fiesta_server src/fiesta_server_node.cc)
target_link_libraries(fiesta_server ${PROJECT_NAME})

add_executable(voxedt_server src/voxedt_server_node.cc)
target_link_libraries(voxedt_server ${PROJECT_NAME})
############
# Install
############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  voxblox_eval
  tsdf_server
  np_tsdf_server
  intensity_server
  simulation_eval
  visualize_tsdf
  voxfield_server
  voxblox_server
  fiesta_server
  voxedt_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
