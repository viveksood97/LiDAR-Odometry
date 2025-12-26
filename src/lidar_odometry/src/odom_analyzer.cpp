#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <algorithm>

namespace lidar_odometry
{

class OdometryAnalyzer : public rclcpp::Node
{
public:
  explicit OdometryAnalyzer(const rclcpp::NodeOptions & options)
  : Node("odometry_analyzer", options)
  {
    odom_est_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_est", rclcpp::SensorDataQoS(),
      std::bind(&OdometryAnalyzer::odomEstCallback, this, std::placeholders::_1));

    odom_gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_sim", rclcpp::SensorDataQoS(),
      std::bind(&OdometryAnalyzer::odomGtCallback, this, std::placeholders::_1));

    traj_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/trajectory_plot", 10);
    err_traj_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/avg_trajectory_error", 10);
    yaw_err_pub_ = create_publisher<std_msgs::msg::Float32>("/yaw_error", 10);

    RCLCPP_INFO(get_logger(), "OdometryAnalyzer component started");
  }

private:
  /* ================= Callbacks ================= */

  void odomEstCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_est_.push_back(msg->pose.pose);
    process();
  }

  void odomGtCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_gt_.push_back(msg->pose.pose);
    process();
  }

  /* ================= Core Logic ================= */

  void process()
  {
    size_t N = std::min(odom_est_.size(), odom_gt_.size());
    if (N <= processed_samples_) return;

    geometry_msgs::msg::PoseArray traj_msg;
    geometry_msgs::msg::PoseArray err_traj_msg;

    traj_msg.header.stamp = now();
    traj_msg.header.frame_id = "map";
    err_traj_msg.header = traj_msg.header;

    for (size_t i = processed_samples_; i < N; ++i)
    {
      const auto & est = odom_est_[i];
      const auto & gt  = odom_gt_[i];

      /* ---- Trajectories ---- */
      traj_msg.poses.push_back(est);
      traj_msg.poses.push_back(gt);

      /* ---- Position error ---- */
      double dx = est.position.x - gt.position.x;
      double dy = est.position.y - gt.position.y;
      double pos_err = std::sqrt(dx * dx + dy * dy);

      sum_sq_pos_error_ += pos_err * pos_err;

      geometry_msgs::msg::Pose err_pose;
      err_pose.position.x = std::abs(dx);
      err_pose.position.y = std::abs(dy);
      err_traj_msg.poses.push_back(err_pose);

      /* ---- Yaw error ---- */
      double yaw_est = getYaw(est.orientation);
      double yaw_gt  = getYaw(gt.orientation);
      double yaw_err = normalizeAngle(yaw_est - yaw_gt);

      sum_abs_yaw_error_ += std::abs(yaw_err);

      processed_samples_++;

      /* ---- Per-step debug ---- */
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Step %zu | PosErr: %.3f m | YawErr: %.2f deg",
        processed_samples_, pos_err, yaw_err * 180.0 / M_PI);
    }

    traj_pub_->publish(traj_msg);
    err_traj_pub_->publish(err_traj_msg);

    /* ---- Publish yaw error mean ---- */
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = sum_abs_yaw_error_ / processed_samples_;
    yaw_err_pub_->publish(yaw_msg);

    /* ---- Print metrics ---- */
    double ate = std::sqrt(sum_sq_pos_error_ / processed_samples_);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[Metrics] Samples: %zu | ATE(RMSE): %.3f m | Mean Yaw Err: %.2f deg",
      processed_samples_, ate, yaw_msg.data * 180.0 / M_PI);
  }

  /* ================= Utilities ================= */

  double getYaw(const geometry_msgs::msg::Quaternion & q) const
  {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalizeAngle(double a) const
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  /* ================= Members ================= */

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_est_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_gt_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr err_traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_err_pub_;

  std::vector<geometry_msgs::msg::Pose> odom_est_;
  std::vector<geometry_msgs::msg::Pose> odom_gt_;

  size_t processed_samples_ = 0;
  double sum_sq_pos_error_ = 0.0;
  double sum_abs_yaw_error_ = 0.0;
};

}  // namespace lidar_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_odometry::OdometryAnalyzer)
