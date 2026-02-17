//--------------------------------------------------------
//
//  L2lidar_ROS2
//  Author: Mark Stegall
//  Module: l2lidar_node.cpp
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
#include "l2lidar_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

//---------------------------------------------------------------------
// L2LidarNode constructor
//---------------------------------------------------------------------
L2LidarNode::L2LidarNode(int argc, char **argv)
    : Node("l2lidar_node")
{
    // --------Declare parameters in config file ----------------
    declare_parameter<std::string>("l2_ip", "192.168.1.62");
    declare_parameter<int>("l2_port", 6101);
    declare_parameter<std::string>("host_ip", "192.168.1.2");
    declare_parameter<int>("host_port", 6201);

    declare_parameter<bool>("enable_l2_time_correction", true);
    declare_parameter<bool>("enable_l2_host_sync", true);
    declare_parameter<int>("l2_sync_rate_ms", 50);
    declare_parameter<bool>("enable_latency_measure", false);

    declare_parameter<std::string>("frame_id", "l2lidar_frame");
    declare_parameter<std::string>("imu_frame_id", "l2lidar_imu");

    declare_parameter<int>("pointcloud_queue_size", 10);
    declare_parameter<int>("imu_queue_size", 10);

    // get parameters from config file

    // get IDs for publish
    // frame_id_ and ime_frame_id_ are private class members
    get_parameter("frame_id", frame_id_);
    get_parameter("imu_frame_id", imu_frame_id_);

    // get UDP parameters
    std::string l2_ip, host_ip;
    int l2_port, host_port;

    get_parameter("l2_ip", l2_ip);
    get_parameter("l2_port", l2_port);
    get_parameter("host_ip", host_ip);
    get_parameter("host_port", host_port);

    // get time correction and timebase syncing parameters
    bool time_corr, host_sync, latency;
    int sync_rate;

    get_parameter("enable_l2_time_correction", time_corr);
    get_parameter("enable_l2_host_sync", host_sync);
    get_parameter("l2_sync_rate_ms", sync_rate);
    get_parameter("enable_latency_measure", latency);

    // --------- Watchdog timer settings---------------
    declare_parameter<int>("watchdog_timeout_ms", 1000);
    get_parameter("watchdog_timeout_ms", watchdog_timeout_ms_);
    last_imu_time_.start();
    last_pc_time_.start();

    connect(&watchdog_timer_, &QTimer::timeout,
            this, &L2LidarNode::watchdogCheck);

    watchdog_timer_.start(500);  // check twice per second

    // ---------------- ROS publishers ----------------
    int pc_queue_size;
    int imu_queue_size;

    get_parameter("pointcloud_queue_size", pc_queue_size);
    get_parameter("imu_queue_size", imu_queue_size);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", imu_queue_size);
    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points", pc_queue_size);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTransform();

    // initialize UDP addresses and ports for sending and receiving UDP packets
    lidar_.LidarSetCmdConfig(
        QString::fromStdString(host_ip), host_port,
        QString::fromStdString(l2_ip), l2_port);

    // enable/disable time base corrections to be applied to point cloud and IMU timestamps
    lidar_.EnableL2TimeCorrection(time_corr);
    // enable/disable host to L2 timebase sync
    lidar_.EnableL2TSsync(host_sync);
    // set the peroidicity of the host to L2 time sync
    lidar_.SetL2TSsyncRate(sync_rate);
    // enable/disable UDP RTT latency measurements
    lidar_.EnableLatencyMeasure(latency);

    // IMU data ready signal/slot connection
    connect(&lidar_, &L2lidar::imuReceived,
            this, &L2LidarNode::onImuReceived);

    // point cloud data ready signal/slot connection
    connect(&lidar_, &L2lidar::PCL3DReceived,
            this, &L2LidarNode::onPointCloudReceived);

    // connect to the L2
    if (!lidar_.ConnectL2()) {
        throw std::runtime_error(
            "L2lidar connected failed: " +
            lidar_.GetLastUDPError().toStdString());
    }

    // ---------------- ROS spin timer ----------------
    connect(&spin_timer_, &QTimer::timeout,
            this, &L2LidarNode::spinOnce);

    spin_timer_.start(5); // 200 Hz spin

    RCLCPP_INFO(get_logger(), "L2Lidar ROS2 node started");
}

//---------------------------------------------------------------------
// L2LidarNode destructor
//---------------------------------------------------------------------

L2LidarNode::~L2LidarNode()
{
    spin_timer_.stop();
    watchdog_timer_.stop();
}

//---------------------------------------------------------------------
// spinOnce
//---------------------------------------------------------------------
void L2LidarNode::spinOnce()
{
    if (!rclcpp::ok())
    {
        QCoreApplication::quit();
        return;
    }

    rclcpp::spin_some(shared_from_this());
}

//---------------------------------------------------------------------
// shutdownNode
//---------------------------------------------------------------------
void L2LidarNode::shutdownNode(const std::string &reason)
{
    RCLCPP_FATAL(get_logger(), "%s", reason.c_str());

    spin_timer_.stop();
    watchdog_timer_.stop();

    lidar_.DisconnectL2();   // if available in your class

    rclcpp::shutdown();
    QCoreApplication::quit();
}

//---------------------------------------------------------------------
// shutdownNode
//---------------------------------------------------------------------
void L2LidarNode::watchdogCheck()
{
    if (!rclcpp::ok())
        return;

    qint64 imu_elapsed = last_imu_time_.elapsed();
    qint64 pc_elapsed  = last_pc_time_.elapsed();

    if (imu_elapsed > watchdog_timeout_ms_)
    {
        shutdownNode("Watchdog timeout: IMU data stalled");
        return;
    }

    if (pc_elapsed > watchdog_timeout_ms_)
    {
        shutdownNode("Watchdog timeout: PointCloud data stalled");
        return;
    }
}

//---------------------------------------------------------------------
// onImuReceived
//---------------------------------------------------------------------
void L2LidarNode::onImuReceived()
{
    last_imu_time_.restart();
    auto imu_packet = lidar_.imu();

    sensor_msgs::msg::Imu msg;
    // time stamp comes from IMU packet not system using now()
    msg.header.stamp.sec = imu_packet.data.info.stamp.sec;
    msg.header.stamp.nanosec = imu_packet.data.info.stamp.nsec;
    msg.header.frame_id = imu_frame_id_;

    // review order of quaternion array
    msg.orientation.x = imu_packet.data.quaternion[0];
    msg.orientation.y = imu_packet.data.quaternion[1];
    msg.orientation.z = imu_packet.data.quaternion[2];
    msg.orientation.w = imu_packet.data.quaternion[3];

    msg.angular_velocity.x = imu_packet.data.angular_velocity[0];
    msg.angular_velocity.y = imu_packet.data.angular_velocity[1];
    msg.angular_velocity.z = imu_packet.data.angular_velocity[2];

    msg.linear_acceleration.x = imu_packet.data.linear_acceleration[0];
    msg.linear_acceleration.y = imu_packet.data.linear_acceleration[1];
    msg.linear_acceleration.z = imu_packet.data.linear_acceleration[2];

    imu_pub_->publish(msg);
}

//---------------------------------------------------------------------
// onPointCloudReceived
//---------------------------------------------------------------------
void L2LidarNode::onPointCloudReceived()
{
    last_pc_time_.restart();
    Frame frame;
    if (!lidar_.ConvertL2data2pointcloud(frame, true, true))
        return;

    if (frame.empty())
        return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;

    // Use first point timestamp as frame timestamp
    double t0 = frame.front().time;
    int64_t sec_part = static_cast<int64_t>(t0);
    int64_t nsec_part = static_cast<int64_t>((t0 - sec_part) * 1e9);
    cloud.header.stamp = rclcpp::Time(sec_part, nsec_part);

    cloud.height = 1;
    cloud.width = frame.size();
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
        "time", 1, sensor_msgs::msg::PointField::FLOAT32
        );

    modifier.resize(frame.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_t(cloud, "time");

    for (const PCpoint &p : std::as_const(frame))
    {
        *iter_x = p.x;
        *iter_y = p.y;
        *iter_z = p.z;
        *iter_i = p.intensity;
        *iter_t = p.time;   // per-point timestamp in seconds

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_i;
        ++iter_t;
    }

    pcl_pub_->publish(cloud);
}

//---------------------------------------------------------------------
// publishStaticTransform
//---------------------------------------------------------------------
void L2LidarNode::publishStaticTransform()
{
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = frame_id_;      // "l2lidar_frame"
    tf_msg.child_frame_id = imu_frame_id_;   // "l2lidar_imu"

    // Identity transform (adjust if you know physical offset)
    tf_msg.transform.translation.x = -0.007698;
    tf_msg.transform.translation.y = -0.014655;
    tf_msg.transform.translation.z = 0.00667;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO(get_logger(), "Published static TF: %s -> %s",
                frame_id_.c_str(), imu_frame_id_.c_str());
}
