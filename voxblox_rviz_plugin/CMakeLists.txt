cmake_minimum_required(VERSION 3.8)
project(voxblox_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(voxblox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(QT_NO_KEYWORDS)

set(SRC_FILES
    src/voxblox_mesh_display.cc
    src/voxblox_multi_mesh_display.cc
    src/voxblox_mesh_visual.cc
    src/material_loader.cc
)

add_library(${PROJECT_NAME} SHARED
    ${SRC_FILES}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rviz_common
    rviz_rendering
    rviz_ogre_vendor
    pluginlib
    rclcpp
    voxblox
    voxblox_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Force our includes to the front to avoid picking up system Ogre
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC
    ${rviz_ogre_vendor_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt5::Core
    Qt5::Widgets
    ${rviz_ogre_vendor_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    FILES plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY icons/
    DESTINATION share/${PROJECT_NAME}/icons
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rviz_common
    rviz_rendering
    rviz_ogre_vendor
    pluginlib
    rclcpp
    voxblox
    voxblox_msgs
)

ament_package()
