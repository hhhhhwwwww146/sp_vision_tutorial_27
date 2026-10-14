#ifndef ADAPTIVE_PURSUIT_CONTROLLER_HPP_
#define ADAPTIVE_PURSUIT_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace adaptive_pursuit_controller
{

class AdaptivePursuitController : public sp_controller_server::ControllerPlugin
{
public:
  AdaptivePursuitController() = default;
  ~AdaptivePursuitController() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  struct Point2d
  {
    double x{0.0};
    double y{0.0};
  };

  struct Projection
  {
    double arc_length{0.0};
    double distance{0.0};
    std::size_t segment_index{0};
  };

  Point2d samplePath(double arc_length) const;
  Projection projectToPath(const Point2d & position) const;
  double interpolateSpeed(double arc_length) const;
  void buildVelocityProfile();
  static double pointDistance(const Point2d & a, const Point2d & b);

  std::string plugin_name_;
  std::string base_frame_id_{"base_link"};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr node_;

  std::vector<Point2d> path_points_;
  std::vector<double> cumulative_lengths_;
  std::vector<double> velocity_profile_;
  double total_length_{0.0};
  double last_progress_{0.0};
  mutable std::mutex plan_mutex_;

  double max_linear_speed_{1.50};
  double max_command_speed_{1.70};
  double min_curve_speed_{0.48};
  double max_lateral_acceleration_{0.90};
  double max_deceleration_{1.55};
  double lookahead_distance_{0.55};
  double lookahead_time_{0.32};
  double max_lookahead_distance_{1.05};
  double curvature_window_{0.30};
  double velocity_feedback_gain_{0.30};
  double stopping_time_{0.72};
  double max_terminal_speed_{0.60};
  double goal_tolerance_{0.08};
};

}  // namespace adaptive_pursuit_controller

#endif  // ADAPTIVE_PURSUIT_CONTROLLER_HPP_
