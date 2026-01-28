# Copilot instructions — Voxfield (ROS -> ROS2 migration focus)

Short goal: help an AI coding agent become productive migrating this ROS1-based codebase to ROS2 Humble and otherwise maintain the project.

- **Big picture**
  - Core C++ library: `voxblox/` (mapping, protobuf usage). Key file: `voxblox/CMakeLists.txt` — builds `libvoxblox` and protobuf-generated code.
  - ROS integration: `voxblox_ros/` contains server classes (TSDF/NP-TSDF/ESDF/FIESTA/etc) and node entrypoints in `src/*_node.cc`.
  - Messages: `voxblox_msgs/msg/*.msg` — converted to ROS2 `rosidl` in `voxblox_msgs/CMakeLists.txt`.
  - RViz UI: `voxblox_rviz_plugin/` — a ROS1 RViz plugin (likely needs full rewrite for RViz2).

- **Service boundaries & data flows**
  - Servers (e.g. `TsdfServer`, `NpTsdfServer`, `VoxbloxServer`) expose topics, services and timers and own maps (`TsdfMap`, `EsdfMap`). See `voxblox_ros/include/voxblox_ros/*.h` and `src/*.cc`.
  - `Transformer` resolves TF or transform topics; many places call `transformer_.lookupTransform(...)` before processing pointclouds.
  - Mesh/messages flow via `voxblox_msgs::Mesh`, `Layer`, etc. These are reused across packages.

- **Project-specific conventions to follow**
  - Staged ROS2 migration is used: add `rclcpp::Node`-based constructors alongside existing ROS1 constructors, then replace internals incrementally. Example: `TsdfServer(const rclcpp::Node::SharedPtr& node)` is added and used by migrated mains (`*_node.cc`). See `voxblox_ros/src/tsdf_server_node.cc` and `voxblox_ros/include/voxblox_ros/tsdf_server.h`.
  - `Transformer` has been converted to a `std::unique_ptr<Transformer>` to decouple construction; prefer deferring TF2 changes until after node wiring.
  - Protobuf use: `voxblox` uses `proto/*.proto` and `protobuf_generate_cpp` (note CMake change from catkin protobuf macros to plain `protobuf_generate_cpp`).

- **Immediate developer workflows (how to build & iterate locally)**
  - Prepare ROS2 Humble environment (example):
    ```bash
    source /opt/ros/humble/setup.bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    # copy repository packages into workspace src
    cd ~/ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ```
  - Build messages first to settle deps: `colcon build --packages-select voxblox_msgs -v`.
  - When iterating on a package: rebuild only it and its dependents: `colcon build --packages-select voxblox_ros voxblox -v`.

- **Migration patterns & concrete replacements**
  - Node init: `ros::init` / `ros::NodeHandle` -> `rclcpp::init` / `rclcpp::Node::SharedPtr` (see `*_node.cc` changes).
  - Publishers/subscribers: `nh.advertise<>` / `nh.subscribe` -> `node->create_publisher<>` / `node->create_subscription<>`.
  - Services: `advertiseService` -> `create_service` with `rclcpp` callback signatures.
  - Timers: `nh.createTimer(ros::Duration, ...)` -> `node->create_wall_timer(...)`.
  - Params: `nh.param(...)` -> `node->declare_parameter(...)` + `node->get_parameter(...)`.
  - TF: `tf::TransformListener` / `tf::TransformBroadcaster` -> `tf2_ros::Buffer` + `tf2_ros::TransformListener` and `tf2_ros::TransformBroadcaster` (non-trivial; update `Transformer` accordingly).

- **Key files to inspect when making changes**
  - `voxblox_msgs/package.xml`, `voxblox_msgs/CMakeLists.txt` — message generation rules.
  - `voxblox/CMakeLists.txt` — core library and protobuf generation.
  - `voxblox_ros/CMakeLists.txt`, `voxblox_ros/package.xml` — ament migration and package deps.
  - Example nodes/servers: `voxblox_ros/src/tsdf_server.cc`, `voxblox_ros/include/voxblox_ros/tsdf_server.h`, `np_tsdf_server.*`, `voxblox_server.*`.
  - RViz plugin: `voxblox_rviz_plugin/` (significant manual porting required for rviz2).

- **Testing & debugging tips**
  - Start by building `voxblox_msgs` then `voxblox` core. Fix CMake/C++ compile errors incrementally.
  - Use `colcon build --event-handlers console_direct+` to get clearer compiler output.
  - For runtime debugging, run migrated node with `ros2 run <pkg> <exe>` and use `ros2 topic echo` / `ros2 service call` / `ros2 topic hz`.

- **Known heavy-lift areas**
  - RViz plugin migration (API differences between rviz and rviz2).
  - `tf` -> `tf2_ros`: transform semantics and stamped time lookups differ.
  - Services and message API changes (service types in `voxblox_msgs` moved to rosidl).

If anything here is unclear or you want the instructions tuned for a specific subtask (e.g., full port of `tsdf_server` internals or RViz plugin rewrite), tell me which target and I will extend this file with step-by-step patches.
