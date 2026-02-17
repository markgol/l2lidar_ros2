//--------------------------------------------------------
//
//  L2lidar_ROS2
//  Author: Mark Stegall
//  Module: l2lidar_node.hpp
//
//	Purpose:
//		This ROS2 package provides an interface between ROS2 and the
//		Untiree L2 4D LiDAR module.  The L2 provides point cloud data
//		 and IMU data as it scans.  The scans are 3D intensity.  It generates
//		point cloud data 300 points at a time (a frame).  The L2 uses an
//		UPD Ethernet interface to send data.  This uses the L2lidar class
//		software package to provide the backend interface to the L2.
//		This is class is structured to be compatible with the formats
//		and interfaces needed for support in the ROS2 packages.
//
//		This ROS2 package publishes the point cloud data and IMU data
//		for ROS2 subscribers.
//
//		- Publishes:
//			/l2lidar/points	(sensor_msgs/PointCloud2)
//			/l2lidar/imu (sensor_msgs/Imu)
//			Static TF transform support
//
//	Implementation
//		This is the ROS2 driver for the Unitree L2 4d LiDAR.
//      This implements a publisher node for IMU and point cloud data.
//      ROS2 nodes sources:
//          src/main.cpp
//          src/l2lidar_node.cpp
//          include/l2lidar_node.hpp
//
//      L2 driver sources (these can be marked read only)
//          src/L2lidar.cpp
//          include/L2lidar.h
//          include/PCpoint.h
//          unitree_lidar_protocol.h
//          untiree_lidar_utilities.h
//
//      Restrictions
//      The sources require Qt6.10.2 or higher.
//      This only uses Qt6 Core and Qt6 Networking.
//      This node is standalone. It does not incorporate any Gui elements.
//      DO NOT USE Qt or ROS2 Gui elements or similar resources in this node.
//
//		Target:	Ubuntu 24.04 systems with ROS2 Jazzy installed
//		Initial target hardware is RPI5 (ARM64)
//		A x86_64 imeplentation will be done after the RPI5 version
//		completed.
//
//		V0.1.0	2026-02-16	Initial package skeleton
//
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <QCoreApplication>
#include <QObject>
#include <QTimer>

#include "L2lidar.h"

class L2LidarNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit L2LidarNode(int argc, char **argv);

private slots:
    void onImuReceived();
    void onPointCloudReceived();
    void spinOnce();

private:
    L2lidar lidar_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

    std::string frame_id_;
    std::string imu_frame_id_;

    QTimer spin_timer_;
};
