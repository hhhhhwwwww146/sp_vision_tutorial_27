#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pid_controller {
namespace {

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

template<typename T>
void declare_and_get(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  T & value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, value);
  }
  value = node->get_parameter(name).get_value<T>();
}

}

void PidController::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  if (!node) {
    throw std::runtime_error("PidController received null node");
  }

  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;

  base_frame_id_ = "base_link";

  declare_and_get(
    node_, plugin_name_ + ".base_frame_id", base_frame_id_);
  declare_and_get(
    node_, plugin_name_ + ".max_linear_speed", max_linear_speed_);
  declare_and_get(
    node_, plugin_name_ + ".min_linear_speed", min_linear_speed_);
  declare_and_get(
    node_, plugin_name_ + ".max_angular_speed", max_angular_speed_);
  declare_and_get(
    node_, plugin_name_ + ".lookahead_distance", lookahead_distance_);
  declare_and_get(
    node_, plugin_name_ + ".goal_tolerance", goal_tolerance_);
  declare_and_get(
    node_, plugin_name_ + ".yaw_tolerance", yaw_tolerance_);
  declare_and_get(
    node_, plugin_name_ + ".rotate_to_path_angle",
    rotate_to_path_angle_);
  declare_and_get(
    node_, plugin_name_ + ".linear_gain", linear_gain_);
  declare_and_get(
    node_, plugin_name_ + ".angular_gain", angular_gain_);

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s path tracker",
    plugin_name_.c_str());
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  last_target_index_ = 0;
}

geometry_msgs::msg::TwistStamped
PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &)
{
  geometry_msgs::msg::TwistStamped cmd;

  cmd.header.stamp = node_->now();
  cmd.header.frame_id = base_frame_id_;

  if (global_plan_.poses.empty()) {
    return cmd;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);

  const auto & goal = global_plan_.poses.back().pose;

  const double goal_dx = goal.position.x - x;
  const double goal_dy = goal.position.y - y;
  const double goal_distance = std::hypot(goal_dx, goal_dy);

  // 到达终点附近后停止平移，必要时调整最终朝向。
  if (goal_distance <= goal_tolerance_) {
    const double goal_yaw = tf2::getYaw(goal.orientation);
    const double yaw_error = normalize_angle(goal_yaw - yaw);

    if (std::abs(yaw_error) > yaw_tolerance_) {
      cmd.twist.angular.z = std::clamp(
        angular_gain_ * yaw_error,
        -max_angular_speed_,
        max_angular_speed_);
    }

    return cmd;
  }

  // 搜索最近的路径点，避免目标索引反复向后跳。
  std::size_t nearest = std::min(
    last_target_index_,
    global_plan_.poses.size() - 1);

  double best_d2 = std::numeric_limits<double>::max();

  for (std::size_t i = nearest;
       i < global_plan_.poses.size(); ++i)
  {
    const auto & p = global_plan_.poses[i].pose.position;

    const double dx = p.x - x;
    const double dy = p.y - y;
    const double d2 = dx * dx + dy * dy;

    if (d2 < best_d2) {
      best_d2 = d2;
      nearest = i;
    } else if (i > nearest + 5 && d2 > best_d2 + 0.20) {
      break;
    }
  }

  last_target_index_ = nearest;

  // 沿路径寻找前视目标点。
  std::size_t target = nearest;

  while (target + 1 < global_plan_.poses.size()) {
    const auto & p =
      global_plan_.poses[target].pose.position;

    if (std::hypot(p.x - x, p.y - y)
        >= lookahead_distance_)
    {
      break;
    }

    ++target;
  }

  const auto & target_pose =
    global_plan_.poses[target].pose.position;

  const double dx = target_pose.x - x;
  const double dy = target_pose.y - y;

  const double target_distance =
    std::max(std::hypot(dx, dy), 1e-4);

  const double heading_error = normalize_angle(
    std::atan2(dy, dx) - yaw);

  // 目标方向偏差过大时，先原地旋转再前进。
  if (std::abs(heading_error) > rotate_to_path_angle_) {
    cmd.twist.angular.z = std::clamp(
      angular_gain_ * heading_error,
      -max_angular_speed_,
      max_angular_speed_);

    return cmd;
  }

  // 前视路径跟踪：根据方向误差计算曲率。
  const double curvature =
    2.0 * std::sin(heading_error) / target_distance;

  // 距离终点越近，线速度越小。
  double speed = std::min(
    max_linear_speed_,
    linear_gain_ * goal_distance);

  // 转向时主动减速。
  speed *= std::max(0.25, std::cos(heading_error));

  speed = std::clamp(
    speed,
    0.0,
    max_linear_speed_);

  if (goal_distance > goal_tolerance_ &&
      speed < min_linear_speed_)
  {
    speed = min_linear_speed_;
  }

  cmd.twist.linear.x = speed;

  cmd.twist.angular.z = std::clamp(
    speed * curvature + angular_gain_ * heading_error,
    -max_angular_speed_,
    max_angular_speed_);

  return cmd;
}

}

PLUGINLIB_EXPORT_CLASS(
  pid_controller::PidController,
  sp_controller_server::ControllerPlugin)
