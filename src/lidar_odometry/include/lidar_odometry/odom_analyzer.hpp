#ifndef LIDAR_ODOMETRY__ODOMETRY_ANALYZER_HPP_
#define LIDAR_ODOMETRY__ODOMETRY_ANALYZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <fstream>
#include <memory>
#include <string>

namespace lidar_odometry
{

class OdometryAnalyzer : public rclcpp::Node
{
public:
  explicit OdometryAnalyzer(const rclcpp::NodeOptions & options);
  ~OdometryAnalyzer() override;

private:
  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry
  > SyncPolicy;

  void synchronizedCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr est_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr gt_msg);

  // Message filter subscribers
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_est_sub_f_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_gt_sub_f_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // CSV output
  std::ofstream csv_file_;

  // --- Publish rate tracking ---
  rclcpp::Time last_est_time_;
  bool has_last_est_time_{false};
};

} // namespace lidar_odometry

#endif // LIDAR_ODOMETRY__ODOMETRY_ANALYZER_HPP_
