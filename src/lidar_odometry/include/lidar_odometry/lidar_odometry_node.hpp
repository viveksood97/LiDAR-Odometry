#ifndef LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
#define LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_

// C++
#include <memory>

// Third-party
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS Interfaces
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

namespace lidar_odometry {

/// 
/// @brief Main node for LiDAR odometry estimation using ICP with point-to-plane constraints.
///
/// This node processes LiDAR point clouds and IMU data to estimate the robot's pose
/// over time. It uses a sliding window of scans to build a local map and performs
/// iterative closest point (ICP) registration with motion prediction.
///
class LidarOdometryNode : public rclcpp::Node {
public:
    ///
    /// @brief Constructor for the LiDAR Odometry Node.
    ///
    /// @param[in] options The node options.
    ///
    explicit LidarOdometryNode(const rclcpp::NodeOptions& options);

private:
   ///
   /// @brief Templated method to downsample a point cloud.
   ///
   /// @tparam T The point type (e.g., pcl::PointXYZ, pcl::PointNormal).
   /// @param[in,out] cloud The point cloud to downsample.
   /// @param[in] leaf_size The size of the voxel grid.
   ///
   template<typename T>
   void downsample(typename pcl::PointCloud<T>::Ptr& cloud, const float leaf_size) {
      if (!cloud || cloud->empty()) return;
      pcl::VoxelGrid<T> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(leaf_size, leaf_size, leaf_size);
      vg.filter(*cloud);
   }
    ///
    /// @brief Initialize the transformation matrix from LiDAR frame to base frame.
    ///
    /// @note This transform is a static transformation from the LiDAR frame to the base 
    /// frame and we should not be using tf_lookup for this. Ideally this lives in a 
    /// config file. I have hard-coded it in this method for now.
    /// @note This transform can be used for both imu and lidar based on looking at the
    /// tf's in foxglove.
    ///
    /// @return The transformation matrix from LiDAR frame to base frame.
    ///
    Eigen::Isometry3f initializeLidarToBaseTransform();
    
    ///
    /// @brief Convert a sensor_msgs::PointCloud2 message to a PCL point cloud and transform it to the base frame.
    ///
    /// @param[in] msg The sensor_msgs::PointCloud2 message to convert.
    ///
    /// @return The converted and transformed point cloud.
    ///
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCLAndTransform(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    ///
    /// @brief Compute normals for a point cloud.
    ///
    /// @param[in] cloud The point cloud to compute normals for.
    ///
    /// @return The point cloud with normals.
    ///
    pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    ///
    /// @brief Get an initial pose guess based on previous pose, imu yaw and time difference.
    ///
    /// @param[in] previous_pose The previous pose.
    /// @param[in] dt The time difference.
    ///
    /// @return The initial pose guess.
    ///
    Eigen::Isometry3d getInitialPoseGuess(const Eigen::Isometry3d& previous_pose, const double dt);

    ///
    /// @brief Get the pose from ICP.
    ///
    /// @note This function modifies the current_pose_base_ private member with the latest pose.
    ///
    /// @param[in] current_with_normals The current point cloud with normals.
    /// @param[in] global_guess The global guess for the pose.
    /// @param[in] dt The time difference.
    ///
    /// @return ICP fitness score (lower is better, < 0.6 considered good).
    ///
    double getPoseICP(
        const pcl::PointCloud<pcl::PointNormal>::Ptr current_with_normals,
        const Eigen::Isometry3d& global_guess,
        const double dt
    );

    ///
    /// @brief Update the local map with the current point cloud.
    ///
    /// @param[in] current_with_normals The current point cloud with normals.
    ///
    void updateLocalMap(const pcl::PointCloud<pcl::PointNormal>::Ptr current_with_normals);

    ///
    /// @brief Callback function for LiDAR messages.
    ///
    /// @param[in] msg The LiDAR point cloud message.
    ///
    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    ///
    /// @brief Publish the odometry message.
    ///
    /// @param[in] timestamp The timestamp for the odometry message.
    ///
    void publishOdometry(const rclcpp::Time& timestamp);

    /// Persistent PCL objects to avoid re-allocation
    /// Normal estimator
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne_;
    /// KD-tree for neighbor search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
    /// ICP algorithm (Point-to-plane)
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp_;
    
    /// Core transforms and state
    /// Static transform: LiDAR frame(lidar_nav) -> base_link
    Eigen::Isometry3f lidar_to_base_transform_;
    /// Current pose in odom frame
    Eigen::Isometry3d current_pose_base_;
    /// Flag indicating if first scan received
    bool initialized_;

    /// Motion model variables
    /// Estimated linear velocity (m/s)
    Eigen::Vector3d last_velocity_linear_;
    /// Flag indicating if velocity estimate is valid
    bool velocity_valid_;
    /// Last LiDAR message timestamp
    rclcpp::Time last_lidar_time_;
    /// Current yaw rate from IMU (rad/s)
    double imu_yaw_rate_;

    /// Map Management
    /// Maximum number of scans to keep in sliding window
    const size_t max_window_size_;
    /// Sliding window of point clouds
    boost::circular_buffer<pcl::PointCloud<pcl::PointNormal>::Ptr> scan_window_;
    /// Local map (accumulated and downsampled point cloud)
    pcl::PointCloud<pcl::PointNormal>::Ptr local_map_;

    // ROS Publishers and Subscribers
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    const rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

} // namespace lidar_odometry

#endif // LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
