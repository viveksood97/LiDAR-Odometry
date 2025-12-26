#include "lidar_odometry/odom_analyzer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <iomanip>

namespace lidar_odometry
{

OdometryAnalyzer::OdometryAnalyzer(const rclcpp::NodeOptions & options)
: Node("odometry_analyzer", options)
{
  std::string filename = this->declare_parameter<std::string>("csv_filepath", "");
  
  if (filename.empty()) {
    RCLCPP_WARN(this->get_logger(), "csv_filepath is empty. No CSV will be written.");
  } else {
    csv_file_.open(filename, std::ios::out);
    if (csv_file_.is_open()) {
      csv_file_ << "timestamp_sec,est_x,est_y,gt_x,gt_y,pos_err,yaw_err\n";
      RCLCPP_INFO(this->get_logger(), "Writing analysis to: %s", filename.c_str());
    }
  }

  odom_est_sub_f_.subscribe(this, "/odom_est", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  odom_gt_sub_f_.subscribe(this, "/odom_sim", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  // 3. Initialize Synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), odom_est_sub_f_, odom_gt_sub_f_);
  
  sync_->registerCallback(std::bind(&OdometryAnalyzer::synchronizedCallback, this, 
                          std::placeholders::_1, std::placeholders::_2));
}

OdometryAnalyzer::~OdometryAnalyzer()
{
  if (csv_file_.is_open()) {
    csv_file_.close();
  }
}

void OdometryAnalyzer::synchronizedCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr est_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr gt_msg)
{
  if (!csv_file_.is_open()) return;

  const auto & est_p = est_msg->pose.pose;
  const auto & gt_p  = gt_msg->pose.pose;

  // Errors
  double dx = est_p.position.x - gt_p.position.x;
  double dy = est_p.position.y - gt_p.position.y;
  double pos_err = std::hypot(dx, dy);

  double est_yaw = tf2::getYaw(est_p.orientation);
  double gt_yaw = tf2::getYaw(gt_p.orientation);
  double yaw_err = std::remainder(est_yaw - gt_yaw, 2.0 * M_PI);

  double ts = rclcpp::Time(est_msg->header.stamp).seconds();

  csv_file_ << std::fixed << std::setprecision(6) 
            << ts << ","
            << est_p.position.x << "," << est_p.position.y << ","
            << gt_p.position.x << "," << gt_p.position.y << ","
            << pos_err << "," << yaw_err << "\n";
}

} // namespace lidar_odometry

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_odometry::OdometryAnalyzer)
