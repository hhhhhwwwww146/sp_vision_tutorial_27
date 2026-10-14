#include "adaptive_pursuit_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace adaptive_pursuit_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

double normalizeAngle(double angle)
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
void declareAndGet(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  T & value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, value);
  }
  value = node->get_parameter(name).get_value<T>();
}

}  // namespace

double AdaptivePursuitController::pointDistance(const Point2d & a, const Point2d & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

void AdaptivePursuitController::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  if (!node) {
    throw std::runtime_error("AdaptivePursuitController received a null node");
  }

  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;

  declareAndGet(node_, plugin_name_ + ".base_frame_id", base_frame_id_);
  declareAndGet(node_, plugin_name_ + ".max_linear_speed", max_linear_speed_);
  declareAndGet(node_, plugin_name_ + ".max_command_speed", max_command_speed_);
  declareAndGet(node_, plugin_name_ + ".min_curve_speed", min_curve_speed_);
  declareAndGet(
    node_, plugin_name_ + ".max_lateral_acceleration", max_lateral_acceleration_);
  declareAndGet(node_, plugin_name_ + ".max_deceleration", max_deceleration_);
  declareAndGet(node_, plugin_name_ + ".lookahead_distance", lookahead_distance_);
  declareAndGet(node_, plugin_name_ + ".lookahead_time", lookahead_time_);
  declareAndGet(
    node_, plugin_name_ + ".max_lookahead_distance", max_lookahead_distance_);
  declareAndGet(node_, plugin_name_ + ".curvature_window", curvature_window_);
  declareAndGet(
    node_, plugin_name_ + ".velocity_feedback_gain", velocity_feedback_gain_);
  declareAndGet(node_, plugin_name_ + ".stopping_time", stopping_time_);
  declareAndGet(node_, plugin_name_ + ".max_terminal_speed", max_terminal_speed_);
  declareAndGet(node_, plugin_name_ + ".goal_tolerance", goal_tolerance_);

  if (max_linear_speed_ <= 0.0 || max_command_speed_ <= 0.0) {
    throw std::runtime_error("Controller speed limits must be positive");
  }
  max_command_speed_ = std::max(max_command_speed_, max_linear_speed_);
  min_curve_speed_ = std::clamp(min_curve_speed_, 0.0, max_linear_speed_);
  max_lateral_acceleration_ = std::max(max_lateral_acceleration_, 0.05);
  max_deceleration_ = std::max(max_deceleration_, 0.05);
  lookahead_distance_ = std::max(lookahead_distance_, 0.05);
  lookahead_time_ = std::max(lookahead_time_, 0.0);
  max_lookahead_distance_ = std::max(max_lookahead_distance_, lookahead_distance_);
  curvature_window_ = std::max(curvature_window_, 0.05);
  velocity_feedback_gain_ = std::max(velocity_feedback_gain_, 0.0);
  stopping_time_ = std::max(stopping_time_, 0.05);
  max_terminal_speed_ = std::clamp(max_terminal_speed_, 0.05, max_linear_speed_);
  goal_tolerance_ = std::max(goal_tolerance_, 0.01);

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s: vmax=%.2f m/s, command_limit=%.2f m/s, lookahead=%.2f+%.2f*v m",
    plugin_name_.c_str(), max_linear_speed_, max_command_speed_,
    lookahead_distance_, lookahead_time_);
}

void AdaptivePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);

  path_points_.clear();
  cumulative_lengths_.clear();
  velocity_profile_.clear();
  total_length_ = 0.0;
  last_progress_ = 0.0;

  path_points_.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const Point2d point{pose.pose.position.x, pose.pose.position.y};
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    if (!path_points_.empty() && pointDistance(path_points_.back(), point) < 1.0e-4) {
      continue;
    }
    path_points_.push_back(point);
  }

  if (path_points_.empty()) {
    return;
  }

  cumulative_lengths_.resize(path_points_.size(), 0.0);
  for (std::size_t i = 1; i < path_points_.size(); ++i) {
    cumulative_lengths_[i] =
      cumulative_lengths_[i - 1] + pointDistance(path_points_[i - 1], path_points_[i]);
  }
  total_length_ = cumulative_lengths_.back();
  buildVelocityProfile();

  RCLCPP_DEBUG(
    node_->get_logger(), "Accepted path with %zu points and %.2f m length",
    path_points_.size(), total_length_);
}

AdaptivePursuitController::Point2d
AdaptivePursuitController::samplePath(double arc_length) const
{
  if (path_points_.empty()) {
    return {};
  }
  if (path_points_.size() == 1 || total_length_ <= kEpsilon) {
    return path_points_.front();
  }

  const double clamped = std::clamp(arc_length, 0.0, total_length_);
  const auto upper = std::upper_bound(
    cumulative_lengths_.begin(), cumulative_lengths_.end(), clamped);
  std::size_t index = 0;
  if (upper != cumulative_lengths_.begin()) {
    index = static_cast<std::size_t>(std::distance(cumulative_lengths_.begin(), upper) - 1);
  }
  index = std::min(index, path_points_.size() - 2);

  const double segment_length = cumulative_lengths_[index + 1] - cumulative_lengths_[index];
  const double ratio = segment_length > kEpsilon ?
    (clamped - cumulative_lengths_[index]) / segment_length : 0.0;

  return {
    path_points_[index].x + ratio * (path_points_[index + 1].x - path_points_[index].x),
    path_points_[index].y + ratio * (path_points_[index + 1].y - path_points_[index].y)};
}

AdaptivePursuitController::Projection
AdaptivePursuitController::projectToPath(const Point2d & position) const
{
  Projection best;
  best.distance = std::numeric_limits<double>::infinity();

  if (path_points_.size() < 2) {
    best.distance = pointDistance(position, path_points_.front());
    return best;
  }

  const double minimum_arc = std::max(0.0, last_progress_ - 0.30);
  for (std::size_t i = 0; i + 1 < path_points_.size(); ++i) {
    if (cumulative_lengths_[i + 1] + kEpsilon < minimum_arc) {
      continue;
    }

    const double dx = path_points_[i + 1].x - path_points_[i].x;
    const double dy = path_points_[i + 1].y - path_points_[i].y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= kEpsilon) {
      continue;
    }

    const double projection_ratio = std::clamp(
      ((position.x - path_points_[i].x) * dx +
      (position.y - path_points_[i].y) * dy) / length_squared,
      0.0, 1.0);
    const double projected_x = path_points_[i].x + projection_ratio * dx;
    const double projected_y = path_points_[i].y + projection_ratio * dy;
    const double distance = std::hypot(position.x - projected_x, position.y - projected_y);

    if (distance < best.distance) {
      best.distance = distance;
      best.segment_index = i;
      best.arc_length = cumulative_lengths_[i] +
        projection_ratio * std::sqrt(length_squared);
    }
  }

  return best;
}

double AdaptivePursuitController::interpolateSpeed(double arc_length) const
{
  if (velocity_profile_.empty()) {
    return 0.0;
  }
  if (velocity_profile_.size() == 1 || total_length_ <= kEpsilon) {
    return velocity_profile_.front();
  }

  const double clamped = std::clamp(arc_length, 0.0, total_length_);
  const auto upper = std::upper_bound(
    cumulative_lengths_.begin(), cumulative_lengths_.end(), clamped);
  std::size_t index = 0;
  if (upper != cumulative_lengths_.begin()) {
    index = static_cast<std::size_t>(std::distance(cumulative_lengths_.begin(), upper) - 1);
  }
  index = std::min(index, velocity_profile_.size() - 2);

  const double ds = cumulative_lengths_[index + 1] - cumulative_lengths_[index];
  const double ratio = ds > kEpsilon ?
    (clamped - cumulative_lengths_[index]) / ds : 0.0;
  return velocity_profile_[index] +
    ratio * (velocity_profile_[index + 1] - velocity_profile_[index]);
}

void AdaptivePursuitController::buildVelocityProfile()
{
  velocity_profile_.assign(path_points_.size(), max_linear_speed_);
  if (path_points_.size() < 2) {
    velocity_profile_.front() = 0.0;
    return;
  }

  for (std::size_t i = 0; i < path_points_.size(); ++i) {
    const double arc = cumulative_lengths_[i];
    Point2d before;
    Point2d center;
    Point2d after;
    if (arc < curvature_window_) {
      before = samplePath(arc);
      center = samplePath(arc + curvature_window_);
      after = samplePath(arc + 2.0 * curvature_window_);
    } else if (total_length_ - arc < curvature_window_) {
      before = samplePath(arc - 2.0 * curvature_window_);
      center = samplePath(arc - curvature_window_);
      after = samplePath(arc);
    } else {
      before = samplePath(arc - curvature_window_);
      center = samplePath(arc);
      after = samplePath(arc + curvature_window_);
    }

    const double before_distance = pointDistance(before, center);
    const double after_distance = pointDistance(center, after);
    double curvature = 0.0;
    if (before_distance > 1.0e-4 && after_distance > 1.0e-4) {
      const double heading_before = std::atan2(center.y - before.y, center.x - before.x);
      const double heading_after = std::atan2(after.y - center.y, after.x - center.x);
      const double span = before_distance + after_distance;
      curvature = std::abs(normalizeAngle(heading_after - heading_before)) /
        std::max(span, 0.05);
    }
    const double curve_speed = std::sqrt(
      max_lateral_acceleration_ / std::max(curvature, 1.0e-4));

    velocity_profile_[i] = std::min(
      max_linear_speed_, std::max(min_curve_speed_, curve_speed));
  }

  velocity_profile_.back() = 0.0;
  for (std::size_t i = velocity_profile_.size() - 1; i > 0; --i) {
    const double segment_length = cumulative_lengths_[i] - cumulative_lengths_[i - 1];
    const double reachable_speed = std::sqrt(
      velocity_profile_[i] * velocity_profile_[i] +
      2.0 * max_deceleration_ * segment_length);
    velocity_profile_[i - 1] = std::min(velocity_profile_[i - 1], reachable_speed);
  }
}

geometry_msgs::msg::TwistStamped
AdaptivePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_->now();
  command.header.frame_id = base_frame_id_;

  std::lock_guard<std::mutex> lock(plan_mutex_);
  if (path_points_.empty()) {
    return command;
  }

  const Point2d position{pose.pose.position.x, pose.pose.position.y};
  const Point2d goal = path_points_.back();
  const double goal_distance = pointDistance(position, goal);
  const double current_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  const double yaw = tf2::getYaw(pose.pose.orientation);

  double desired_x = 0.0;
  double desired_y = 0.0;

  if (path_points_.size() >= 2 && total_length_ > kEpsilon) {
    const Projection projection = projectToPath(position);
    last_progress_ = std::max(last_progress_, projection.arc_length);

    if (goal_distance > goal_tolerance_) {
      const double lookahead = std::clamp(
        lookahead_distance_ + lookahead_time_ * current_speed,
        lookahead_distance_, max_lookahead_distance_);
      const Point2d target = samplePath(last_progress_ + lookahead);
      const double dx = target.x - position.x;
      const double dy = target.y - position.y;
      const double target_distance = std::hypot(dx, dy);

      if (target_distance > kEpsilon) {
        double desired_speed = interpolateSpeed(last_progress_);
        const double remaining_path = std::max(0.0, total_length_ - last_progress_);
        desired_speed = std::min(
          desired_speed, std::max(remaining_path, goal_distance) / stopping_time_);
        desired_x = desired_speed * dx / target_distance;
        desired_y = desired_speed * dy / target_distance;

        // Once the lookahead reaches the endpoint, switch to a terminal
        // position controller. This also pulls the robot back after a small
        // overshoot instead of leaving the speed profile at exactly zero.
        if (remaining_path < lookahead) {
          const double terminal_speed = std::min(
            max_terminal_speed_, goal_distance / stopping_time_);
          desired_x = terminal_speed * (goal.x - position.x) /
            std::max(goal_distance, kEpsilon);
          desired_y = terminal_speed * (goal.y - position.y) /
            std::max(goal_distance, kEpsilon);
        }
      }
    }

    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "progress=%.2f/%.2f m cross_track=%.3f m goal=%.3f m speed=%.2f m/s",
      last_progress_, total_length_, projection.distance, goal_distance, current_speed);
  }

  // The simulator reports velocity in map/odom axes. A small velocity feedback term
  // compensates the plant lag, and becomes active braking when desired velocity is zero.
  double command_map_x = desired_x +
    velocity_feedback_gain_ * (desired_x - velocity.linear.x);
  double command_map_y = desired_y +
    velocity_feedback_gain_ * (desired_y - velocity.linear.y);

  const double command_norm = std::hypot(command_map_x, command_map_y);
  if (command_norm > max_command_speed_) {
    const double scale = max_command_speed_ / command_norm;
    command_map_x *= scale;
    command_map_y *= scale;
  }

  // cmd_vel is expressed in the rotating base_link frame, while the planned path is in map.
  // Rotate the desired map-frame vector into base_link; angular.z is ignored by this simulator.
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  command.twist.linear.x = cos_yaw * command_map_x + sin_yaw * command_map_y;
  command.twist.linear.y = -sin_yaw * command_map_x + cos_yaw * command_map_y;
  command.twist.angular.z = 0.0;

  return command;
}

}  // namespace adaptive_pursuit_controller

PLUGINLIB_EXPORT_CLASS(
  adaptive_pursuit_controller::AdaptivePursuitController,
  sp_controller_server::ControllerPlugin)
