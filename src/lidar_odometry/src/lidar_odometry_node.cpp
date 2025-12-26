#include "lidar_odometry/lidar_odometry_node.hpp"

namespace lidar_odometry {

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions& options)
: Node("lidar_odometry_node", options)
, lidar_to_base_transform_(initializeLidarToBaseTransform())
, current_pose_base_(Eigen::Isometry3d::Identity())
, initialized_{false}
, last_velocity_linear_(Eigen::Vector3d::Zero())
, velocity_valid_{false}
, last_lidar_time_(this->now())
, imu_yaw_rate_(0.0)
, max_window_size_{15}
, scan_window_(max_window_size_)
, local_map_(std::make_shared<pcl::PointCloud<pcl::PointNormal>>())
, odom_pub_(this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom_est", 
      rclcpp::SensorDataQoS()
  ))
, imu_sub_(this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/amr/imu",
      rclcpp::SensorDataQoS(), 
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          imu_yaw_rate_ = msg->angular_velocity.z;
      }
  ))
, lidar_sub_(this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/amr/lidar",
      rclcpp::SensorDataQoS(),
      std::bind(&LidarOdometryNode::lidarCallback, this, std::placeholders::_1)
  ))
{
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    // Pre-configure the estimator
    ne_.setNumberOfThreads(0); // Use all cores
    ne_.setSearchMethod(tree_);
    ne_.setKSearch(12);

    // Pre-configure the ICP
    icp_.setMaxCorrespondenceDistance(0.8);
    icp_.setTransformationEpsilon(1e-7);
    icp_.setMaximumIterations(50);

    RCLCPP_INFO(this->get_logger(), "Lidar Odometry Node has been started.");
}

Eigen::Isometry3f LidarOdometryNode::initializeLidarToBaseTransform() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T(0, 3) = -0.376; 
    T(1, 3) = -0.326; 
    T(2, 3) = 1.394;

    return Eigen::Isometry3f(T.cast<float>());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometryNode::convertToPCLAndTransform(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, lidar_to_base_transform_);
    return cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr LidarOdometryNode::computeNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) 
{
    auto cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    
    ne_.setInputCloud(cloud);
    ne_.compute(*cloud_with_normals);
    pcl::copyPointCloud(*cloud, *cloud_with_normals);
    
    return cloud_with_normals;
}

Eigen::Isometry3d LidarOdometryNode::getInitialPoseGuess(const Eigen::Isometry3d& previous_pose, const double dt) {
    Eigen::Isometry3d prediction = Eigen::Isometry3d::Identity();
    prediction.rotate(Eigen::AngleAxisd(imu_yaw_rate_ * dt, Eigen::Vector3d::UnitZ()));
    if (velocity_valid_ && dt < 0.5) { // Ensure dt is sane
        prediction.pretranslate(last_velocity_linear_ * dt);
    }
    return previous_pose * prediction;
}

double LidarOdometryNode::getPoseICP(const pcl::PointCloud<pcl::PointNormal>::Ptr current_with_normals, const Eigen::Isometry3d& global_guess, const double dt) {
    icp_.setInputSource(current_with_normals);
    icp_.setInputTarget(local_map_);
    pcl::PointCloud<pcl::PointNormal> aligned;
    icp_.align(aligned, global_guess.matrix().cast<float>());
    double score = icp_.getFitnessScore();
    Eigen::Isometry3d previous_pose = current_pose_base_;

    if (icp_.hasConverged() && score < 0.6) {
        current_pose_base_ = Eigen::Isometry3d(icp_.getFinalTransformation().cast<double>());
        if (dt > 0.001) {
            last_velocity_linear_ = (current_pose_base_.translation() - previous_pose.translation()) / dt;
            velocity_valid_ = true;
        }
    } else {
        RCLCPP_DEBUG(this->get_logger(), "ICP Failed/Degenerate (Score: %.3f). Using Prediction.", score);
        Eigen::Isometry3d icp_res(icp_.getFinalTransformation().cast<double>());
        double yaw = atan2(icp_res.rotation()(1,0), icp_res.rotation()(0,0));
        current_pose_base_.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        current_pose_base_.translation() = global_guess.translation();
    }

    // 7. Pose Constraints (2D optimization)
    current_pose_base_.translation().z() = 0.0;
    double final_yaw = atan2(current_pose_base_.rotation()(1,0), current_pose_base_.rotation()(0,0));
    current_pose_base_.linear() = Eigen::AngleAxisd(final_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return score;
}

void LidarOdometryNode::updateLocalMap(const pcl::PointCloud<pcl::PointNormal>::Ptr current_with_normals) {
    auto scan_world = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::transformPointCloudWithNormals(*current_with_normals, *scan_world, current_pose_base_.matrix().cast<float>());
    
    scan_window_.push_back(scan_world);
    
    local_map_->clear();
    size_t total_points = 0;
    for (const auto& s : scan_window_) total_points += s->size();
    local_map_->reserve(total_points);
    for (const auto& s : scan_window_) *local_map_ += *s;
    
    downsample<pcl::PointNormal>(local_map_, 0.15f);
}

void LidarOdometryNode::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    rclcpp::Time start = this->now();
    
    // 1. Convert and Pre-filter
    auto current_scan = convertToPCLAndTransform(msg);
    
    downsample<pcl::PointXYZ>(current_scan, 0.12f);

    // 2. Compute Normals for CURRENT SCAN ONLY
    auto current_with_normals = computeNormals(current_scan);

    // 3. Handle First Frame
    if (!initialized_) {
        scan_window_.push_back(current_with_normals);
        *local_map_ = *current_with_normals;
        last_lidar_time_ = msg->header.stamp;
        initialized_ = true;
        return;
    }

    // 4. Motion Prediction
    double dt = (rclcpp::Time(msg->header.stamp) - last_lidar_time_).seconds();
    Eigen::Isometry3d global_guess = getInitialPoseGuess(current_pose_base_, dt);

    // 5. Point-to-Plane ICP
    double score = getPoseICP(current_with_normals, global_guess, dt);


    // 8. Efficient Map Update
    updateLocalMap(current_with_normals);

    // 9. Finish
    publishOdometry(msg->header.stamp);
    last_lidar_time_ = msg->header.stamp;

    long long ms = (this->now() - start).to_chrono<std::chrono::milliseconds>().count();
    RCLCPP_DEBUG_EXPRESSION(this->get_logger(), ms > 100, "High Latency Detected - Odom Loop: %lld ms", ms);
}

void LidarOdometryNode::publishOdometry(const rclcpp::Time& timestamp) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    auto q = Eigen::Quaterniond(current_pose_base_.rotation());
    odom.pose.pose.position.x = current_pose_base_.translation().x();
    odom.pose.pose.position.y = current_pose_base_.translation().y();
    odom.pose.pose.position.z = 0.0;
    
    odom.pose.pose.orientation.x = q.x(); 
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z(); 
    odom.pose.pose.orientation.w = q.w();
    
    odom_pub_->publish(odom);
}

} // namespace lidar_odometry

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_odometry::LidarOdometryNode)
