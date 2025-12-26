#ifndef LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
#define LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <deque>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp_nl.h> // Includes Point-to-Plane
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

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
    Eigen::Isometry3f initializeLidarToBaseTransform();
    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void publishOdometry(const rclcpp::Time& timestamp);
    
    // Core transforms and state
    Eigen::Isometry3f lidar_to_base_transform_;
    Eigen::Isometry3d current_pose_base_;   
    bool initialized_ = false;

    // Motion model variables
    Eigen::Vector3d last_velocity_linear_ = Eigen::Vector3d::Zero();
    bool velocity_valid_ = false;
    rclcpp::Time last_lidar_time_;
    double imu_yaw_rate_ = 0.0;

    // I should use a circular buffer here instead of deque
    std::deque<pcl::PointCloud<pcl::PointNormal>::Ptr> scan_window_;
    pcl::PointCloud<pcl::PointNormal>::Ptr local_map_;
    const size_t max_window_size_ = 15;

    // ROS 
    const rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

} // namespace lidar_odometry

#endif // LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_