#include "voxblox_ros/np_tsdf_server.h"

#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<rclcpp::Node>("voxfield");
  voxblox::NpTsdfServer server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
