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
    ///
    /// @brief Initialize the transformation matrix from LiDAR frame to base frame.
    ///
    /// @note This transform is a static transformation from the LiDAR frame to the base 
    /// frame and we should not be using tf_lookup for this. Ideally this lives in a 
    /// config file. I have hard-coded it in this method for now.
    /// @note This transform will we used for both imu and lidar based on looking at the
    /// tf's in foxglove.
    ///
    /// @return The transformation matrix from LiDAR frame to base frame.
    ///
    Eigen::Isometry3d initializeLidarToBaseTransform();
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    Eigen::Isometry3d lidar_to_base_transform_;
    const rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

} // namespace lidar_odometry

#endif  // VEHICLE_CFG__STATIC_TF_BROADCASTER_HPP_
