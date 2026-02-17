//--------------------------------------------------------
//
//  L2lidar_ROS2
//  Author: Mark Stegall
//  Module: main.cpp
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
#include <QCoreApplication>
#include <csignal>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include "l2lidar_node.hpp"

static std::atomic_bool g_shutdown_requested{false};

void signalHandler(int signal)
{
    if (!g_shutdown_requested.exchange(true))
    {
        RCLCPP_WARN(rclcpp::get_logger("l2lidar_node"),
                    "Shutdown signal received (%d)", signal);

        QMetaObject::invokeMethod(
            QCoreApplication::instance(),
            "quit",
            Qt::QueuedConnection);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    // this allows orderly shutodwn when ctrl+c is used
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    int ret = EXIT_SUCCESS;

    try
    {
        auto node = std::make_shared<L2LidarNode>(argc, argv);
        ret = app.exec();   // Qt event loop
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("l2lidar_node"),
                     "Fatal startup error: %s", e.what());
        ret = EXIT_FAILURE;
    }

    if (rclcpp::ok())
        rclcpp::shutdown();

    return ret;
}
