# SLAM TSDF Reconstruction

这是基于 `NpTsdfServer` 创建的新类 `SlamTsdfReconstruction`，专门用于房间的三维重建。

## 主要特性

### TSDF 重建功能：
- **NP-TSDF 算法**：使用法向量增强的 TSDF 重建
- **点云处理**：支持 RGB、强度和标签点云
- **预处理**：点云到距离图像的转换和法向量估计
- **ICP 姿态优化**：基于 ICP 的姿态精化
- **网格生成**：实时网格重建和可视化
- **传感器支持**：支持 LiDAR 和相机传感器
- **内存管理**：自动清理远距离块以节省内存

## 类结构

```cpp
class SlamTsdfReconstruction {
public:
    // 构造函数
    SlamTsdfReconstruction(const rclcpp::Node::SharedPtr& node);
    
    // 直接调用接口 - 简单友好
    void processPointcloudDirect(
        const Pointcloud& points_C, 
        const Transformation& T_G_C);
    
    // 其他功能
    void updateMesh();
    void publishPointclouds();
    void publishMap(bool reset_remote_map = false);
    
    // 地图访问器
    std::shared_ptr<TsdfMap> getTsdfMapPtr();
};
```

## 使用方法

### 1. 直接调用接口（推荐）

```cpp
#include "voxblox_ros/slam_tsdf_reconstruction.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("slam_reconstruction");
    
    // 创建 SLAM 重建服务器
    voxblox::SlamTsdfReconstruction slam_reconstruction(node);
    
    // 处理来自cartographer的轨迹数据
    for (auto& trajectory_point : trajectory_data) {
        voxblox::Pointcloud points_C;      // 从cartographer获取的点云
        voxblox::Transformation T_G_C;     // 从cartographer获取的位姿
        
        // 简单调用，只需要点云和位姿
        slam_reconstruction.processPointcloudDirect(points_C, T_G_C);
    }
    
    // 生成最终网格
    slam_reconstruction.generateMesh();
    
    return 0;
}
```

### 2. 基本ROS节点使用

```cpp
#include "voxblox_ros/slam_tsdf_reconstruction.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("slam_reconstruction");
    
    // 创建 SLAM 重建服务器
    voxblox::SlamTsdfReconstruction slam_reconstruction(node);
    
    rclcpp::spin(node);
    return 0;
}
```

### 2. 参数配置

在 ROS2 参数文件中配置：

```yaml
slam_reconstruction:
  ros__parameters:
    # TSDF 参数
    tsdf_voxel_size: 0.05
    tsdf_voxels_per_side: 16
    
    # 传感器参数
    sensor_is_lidar: true
    width: 1024
    height: 64
    
    # 发布设置
    publish_pointclouds: true
    publish_slices: true
    
    # 更新频率
    update_mesh_every_n_sec: 1.0
```

### 3. 话题接口（可选）

**注意：当前版本已移除话题订阅功能和freespace点云支持，专注于直接调用接口。**

#### 输出话题：
- `/mesh` - 重建的网格
- `/tsdf_pointcloud` - TSDF 点云
- `/surface_pointcloud` - 表面点云
- `/tsdf_slice` - TSDF 切片
- `/gsdf_slice` - 梯度切片

#### 服务：
- `/generate_mesh` - 生成网格
- `/save_map` - 保存地图
- `/load_map` - 加载地图
- `/clear_map` - 清除地图

## 编译

确保在 CMakeLists.txt 中添加新的源文件：

```cmake
add_library(${PROJECT_NAME}
  # ... 其他源文件
  src/slam_tsdf_reconstruction.cc
)

add_executable(slam_tsdf_reconstruction_node
  src/slam_tsdf_reconstruction_node.cc
)

target_link_libraries(slam_tsdf_reconstruction_node
  ${PROJECT_NAME}
)
```

## 主要改进

1. **极简接口**：只需要传入点云数据和位姿变换，无需任何其他参数
2. **移除复杂功能**：去除了freespace点云、话题订阅等不必要的功能
3. **内部自动处理**：颜色和法向量会在内部自动生成和计算
4. **复用现有逻辑**：直接调用原有的 `processPointCloudMessageAndInsert` 方法
5. **专注TSDF重建**：保留了原有 NpTsdfServer 的核心TSDF重建功能

## 注意事项

- 确保所有依赖库已正确安装
- 根据实际传感器调整参数配置
- 监控内存使用，大场景可能需要调整体素大小
- 使用 `verbose` 参数可以获得详细的运行信息

## 示例启动

```bash
ros2 run voxblox_ros slam_tsdf_reconstruction_node --ros-args --params-file config.yaml
```