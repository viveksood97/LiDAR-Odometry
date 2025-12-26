#include "lidar_odometry/lidar_odometry_node.hpp"

namespace lidar_odometry
{

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions& options)
: Node("lidar_odometry_node", options)
  , imu_sub_(this->create_subscription<sensor_msgs::msg::Imu>(
        "/livox/amr/imu",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarOdometryNode::imuCallback, this, std::placeholders::_1)
    ))
  , lidar_sub_(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/amr/lidar",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarOdometryNode::lidarCallback, this, std::placeholders::_1)
    )),
    odom_pub_(this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom_est"
        , 10
    ))
{
    RCLCPP_INFO(this->get_logger(), "Lidar Odometry Node has been started.");
}

void LidarOdometryNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received IMU data.");
}

void LidarOdometryNode::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received LiDAR point cloud data.");
};


} // namespace lidar_odometry

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_odometry::LidarOdometryNode)
