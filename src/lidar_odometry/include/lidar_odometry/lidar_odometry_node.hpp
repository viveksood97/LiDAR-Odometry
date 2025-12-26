#ifndef LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
#define LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_

// C++
#include <Eigen/Dense>

// ROS Interfaces
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

namespace lidar_odometry {

class LidarOdometryNode : public rclcpp::Node {
 public:
    explicit LidarOdometryNode(const rclcpp::NodeOptions& options);
 private:
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    const rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

} // namespace lidar_odometry

#endif  // VEHICLE_CFG__STATIC_TF_BROADCASTER_HPP_
