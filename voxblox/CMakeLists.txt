#########
cmake_minimum_required(VERSION 3.5)
project(voxblox)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(minkindr REQUIRED)
find_package(voxblox_msgs REQUIRED)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

##########
# Protobuf
##########
set(PROTO_FILES
  proto/voxblox/Block.proto
  proto/voxblox/Layer.proto
)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Ensure generated protobuf headers are visible to targets.
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)

####################
# Source files
####################
set(${PROJECT_NAME}_SRCS
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/integrator/np_tsdf_integrator.cc
  src/integrator/esdf_voxfield_integrator.cc
  src/integrator/esdf_occ_fiesta_integrator.cc
  src/integrator/esdf_occ_edt_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
  src/utils/neighbor_tools_ex.cc
)

#############
# Libraries
#############
add_library(${PROJECT_NAME}_proto ${PROTO_SRCS} ${PROTO_HDRS})

target_include_directories(${PROJECT_NAME}_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
if(TARGET minkindr)
  target_include_directories(${PROJECT_NAME} PUBLIC $<TARGET_PROPERTY:minkindr,INTERFACE_INCLUDE_DIRECTORIES>)
elseif(DEFINED minkindr_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${minkindr_INCLUDE_DIRS})
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_proto ${Protobuf_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

############
# Install
############
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_proto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
ament_export_dependencies(Protobuf minkindr)
# Export include directories so other packages (e.g. voxblox_ros) can
# find generated protobuf headers (Block.pb.h, Layer.pb.h).
ament_export_include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

install(FILES ${PROTO_HDRS}
        DESTINATION include/${PROJECT_NAME})
ament_package()
