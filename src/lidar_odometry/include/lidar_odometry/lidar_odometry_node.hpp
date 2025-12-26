#ifndef LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
#define LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

#include <boost/circular_buffer.hpp>

namespace lidar_odometry {

class LidarOdometryNode : public rclcpp::Node {
public:
    explicit LidarOdometryNode(const rclcpp::NodeOptions& options);

private:
   template<typename T>
   void downsample(typename pcl::PointCloud<T>::Ptr & cloud, const float leaf_size) {
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCLAndTransform(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    Eigen::Isometry3d getInitialPoseGuess(const Eigen::Isometry3d& previous_pose, const double dt);
    double getPoseICP(const pcl::PointCloud<pcl::PointNormal>::Ptr current_with_normals, const Eigen::Isometry3d& global_guess, const double dt);
    void updateLocalMap(const pcl::PointCloud<pcl::PointNormal>::Ptr current_with_normals);
    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void publishOdometry(const rclcpp::Time& timestamp);

    // Persistent PCL objects to avoid re-allocation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp_;
    
    // Core transforms and state
    Eigen::Isometry3f lidar_to_base_transform_;
    Eigen::Isometry3d current_pose_base_;   
    bool initialized_;

    // Motion model variables
    Eigen::Vector3d last_velocity_linear_;
    bool velocity_valid_;
    rclcpp::Time last_lidar_time_;
    double imu_yaw_rate_;

    const size_t max_window_size_;
    boost::circular_buffer<pcl::PointCloud<pcl::PointNormal>::Ptr> scan_window_;
    pcl::PointCloud<pcl::PointNormal>::Ptr local_map_;

    // ROS 
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    const rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

} // namespace lidar_odometry

#endif // LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
