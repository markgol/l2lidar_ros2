# l2lidar_ros2

ROS2 Jazzy driver for the Unitree L2 LiDAR using a custom Qt6-based hardware interface.

## Features
- Publishes:
  - `/l2lidar/points` (sensor_msgs/PointCloud2)
  - `/l2lidar/imu` (sensor_msgs/Imu)
  Publishes:
/imu/data (sensor_msgs::msg::Imu)
/points (sensor_msgs::msg::PointCloud2)

- Device timestamp synchronization
- Host ? L2 clock sync (configurable rate)
- Qt6 UDP backend
- Static TF transform support

## Parameters

| Name | Default | Description |
|------|---------|-------------|
| frame_id | lidar_link | PointCloud frame |
| imu_frame_id | imu_link | IMU frame |
| host_ip | 192.168.1.2 | Host IP |
| host_port | 6201 | Host port |
| lidar_ip | 192.168.1.62 | L2 IP |
| lidar_port | 6101 | L2 port |
| enable_time_correction | true | Enable L2 timestamp correction |
| enable_time_sync | true | Sync host time to L2 |
| l2_time_sync_rate_ms | 1000 | Sync interval (ms) |
| imu_adjust | true | Apply IMU correction |

## Build

```bash
cd ~/ros2_ws/src
git clone <your_repo_url>
cd ..
colcon build --packages-select l2lidar_ros2
source install/setup.bash



l2lidar_ros2/
+-- CMakeLists.txt
+-- package.xml
+-- launch/
¦   +-- l2lidar.launch.py
+-- params/
¦   +-- l2lidar.yaml
+-- include/l2lidar_ros2/
¦   +-- l2lidar_node.hpp
+-- src/
    +-- l2lidar_node.cpp
    +-- main.cpp
+-- rviz
	+-- rvizl2lidar.rviz