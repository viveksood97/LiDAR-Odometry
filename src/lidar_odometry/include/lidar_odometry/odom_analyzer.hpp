#ifndef LIDAR_ODOMETRY__ODOMETRY_ANALYZER_HPP_
#define LIDAR_ODOMETRY__ODOMETRY_ANALYZER_HPP_

// C++
#include <fstream>
#include <memory>
#include <string>

// Third-party
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// ROS Interfaces
#include <nav_msgs/msg/odometry.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

namespace lidar_odometry {

///
/// @brief Node for analyzing odometry performance by comparing estimated and ground truth odometry.
///
class OdometryAnalyzer : public rclcpp::Node {
public:
    ///
    /// @brief Constructor for the Odometry Analyzer Node.
    ///
    /// @param[in] options The node options.
    ///
    explicit OdometryAnalyzer(const rclcpp::NodeOptions& options);

    ///
    /// @brief Destructor for the Odometry Analyzer Node.
    ///
    /// @note This destructor ensures proper closing of the CSV file.
    ///
    ~OdometryAnalyzer() override;

private:
    /// Alias for the synchronization policy for the message filters.
    typedef message_filters::sync_policies::ApproximateTime <
        nav_msgs::msg::Odometry,
        nav_msgs::msg::Odometry
    > SyncPolicy;

    ///
    /// @brief Callback function for synchronized odometry messages.
    ///
    /// @param[in] est_msg The estimated odometry message.
    /// @param[in] gt_msg The ground truth odometry message.
    ///
    void synchronizedCallback(
        const nav_msgs::msg::Odometry::ConstSharedPtr est_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr gt_msg
    );

    // Message filter subscribers
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_est_sub_f_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_gt_sub_f_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // CSV output
    std::ofstream csv_file_;

    // Publish rate tracking
    rclcpp::Time last_est_time_;
    bool has_last_est_time_;
};

} // namespace lidar_odometry

#endif // LIDAR_ODOMETRY__ODOMETRY_ANALYZER_HPP_