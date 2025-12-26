#include "lidar_odometry/lidar_odometry_node.hpp"

namespace lidar_odometry
{

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions& options)
: Node("lidar_odometry_node", options)
  , lidar_to_base_transform_(initializeLidarToBaseTransform())
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

Eigen::Isometry3d LidarOdometryNode::initializeLidarToBaseTransform()
{
    Eigen::Matrix4d T_lidar_to_base = Eigen::Matrix4d::Identity();

    // Rotation: -180 degrees around Z
    Eigen::AngleAxisd yaw_rotation(-M_PI, Eigen::Vector3d::UnitZ());
    T_lidar_to_base.block<3,3>(0,0) = yaw_rotation.toRotationMatrix();

    // Translation
    T_lidar_to_base(0, 3) = -0.376;
    T_lidar_to_base(1, 3) = -0.326;
    T_lidar_to_base(2, 3) =  1.394;

    return Eigen::Isometry3d(T_lidar_to_base);
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
