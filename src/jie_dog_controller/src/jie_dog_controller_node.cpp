#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class JieDogController : public rclcpp::Node
{
public:
  JieDogController()
  : Node("jie_dog_controller"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.4);
    if (lookahead_distance_ <= 0.0) {
      throw std::invalid_argument("lookahead_distance must be positive");
    }

    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&JieDogController::path_callback, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odin1/odometry", 10,
      std::bind(&JieDogController::odometry_callback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<std_msgs::msg::String>(
      "/dog/state", 10,
      std::bind(&JieDogController::state_callback, this, std::placeholders::_1));
    debug_publisher_ = create_publisher<std_msgs::msg::String>("/dog/control_debug", 10);
    desired_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/dog/desired_cmd_normalized", 10);
    tracking_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&JieDogController::tracking_update, this));
  }

private:
  struct DebugValues
  {
    std::string state;
    std::string path_frame;
    double robot_map_x = 0.0;
    double robot_map_y = 0.0;
    double robot_map_yaw = 0.0;
    int nearest_index = -1;
    int target_index = -1;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_local_x = 0.0;
    double target_local_y = 0.0;
    double distance = 0.0;
    double yaw_error = 0.0;
    double desired_forward_cmd = 0.0;
    double desired_yaw_cmd = 0.0;
  };

  void state_callback(const std_msgs::msg::String::SharedPtr message)
  {
    last_dog_state_ = message->data;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr message)
  {
    last_path_ = *message;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_odometry_ = *message;
    // This is receipt time on the local ROS clock, not the device timestamp in the message.
    last_odom_receipt_time_ = now();
  }

  void tracking_update()
  {
    DebugValues debug;
    if (!last_path_) {
      debug.state = "NO_PATH";
      publish_zero_desired(debug);
      return;
    }

    debug.path_frame = last_path_->header.frame_id;
    if (debug.path_frame.empty()) {
      debug.state = "WAITING_TF";
      publish_zero_desired(debug);
      return;
    }

    if (!latest_odometry_ || !last_odom_receipt_time_) {
      debug.state = "WAITING_ODOM";
      publish_zero_desired(debug);
      return;
    }

    const auto odom_age = now() - *last_odom_receipt_time_;
    if (odom_age > rclcpp::Duration::from_seconds(0.5)) {
      debug.state = "STALE_ODOM";
      publish_zero_desired(debug);
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose_odom;
    robot_pose_odom.header = latest_odometry_->header;
    robot_pose_odom.pose = latest_odometry_->pose.pose;

    geometry_msgs::msg::PoseStamped robot_pose_map;
    try {
      robot_pose_map = tf_buffer_.transform(
        robot_pose_odom, debug.path_frame, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & exception) {
      debug.state = "WAITING_TF";
      publish_zero_desired(debug);
      RCLCPP_DEBUG(get_logger(), "Waiting for %s <- %s TF: %s", debug.path_frame.c_str(),
        robot_pose_odom.header.frame_id.c_str(), exception.what());
      return;
    }

    debug.robot_map_x = robot_pose_map.pose.position.x;
    debug.robot_map_y = robot_pose_map.pose.position.y;
    debug.robot_map_yaw = tf2::getYaw(robot_pose_map.pose.orientation);

    double nearest_distance = std::numeric_limits<double>::infinity();
    bool has_valid_waypoint = false;
    std::size_t nearest_index = 0;
    std::size_t last_valid_index = 0;
    for (std::size_t index = 0; index < last_path_->poses.size(); ++index) {
      const auto & waypoint = last_path_->poses[index].pose.position;
      if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y)) {
        continue;
      }

      has_valid_waypoint = true;
      last_valid_index = index;
      const double waypoint_distance = std::hypot(
        waypoint.x - debug.robot_map_x, waypoint.y - debug.robot_map_y);
      if (waypoint_distance < nearest_distance) {
        nearest_distance = waypoint_distance;
        nearest_index = index;
      }
    }

    if (!has_valid_waypoint) {
      debug.state = "NO_VALID_TARGET";
      publish_zero_desired(debug);
      return;
    }

    std::size_t target_index = last_valid_index;
    for (std::size_t index = nearest_index; index < last_path_->poses.size(); ++index) {
      const auto & waypoint = last_path_->poses[index].pose.position;
      if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y)) {
        continue;
      }
      const double waypoint_distance = std::hypot(
        waypoint.x - debug.robot_map_x, waypoint.y - debug.robot_map_y);
      if (waypoint_distance >= lookahead_distance_) {
        target_index = index;
        break;
      }
    }

    const auto & target = last_path_->poses[target_index].pose.position;
    debug.nearest_index = static_cast<int>(nearest_index);
    debug.target_index = static_cast<int>(target_index);
    debug.target_x = target.x;
    debug.target_y = target.y;

    const double dx = debug.target_x - debug.robot_map_x;
    const double dy = debug.target_y - debug.robot_map_y;
    const double cos_yaw = std::cos(debug.robot_map_yaw);
    const double sin_yaw = std::sin(debug.robot_map_yaw);
    debug.target_local_x = cos_yaw * dx + sin_yaw * dy;
    debug.target_local_y = -sin_yaw * dx + cos_yaw * dy;
    debug.distance = std::hypot(debug.target_local_x, debug.target_local_y);
    debug.yaw_error = std::atan2(debug.target_local_y, debug.target_local_x);

    if (target_index == last_valid_index && debug.distance < 0.1) {
      debug.state = "GOAL_REACHED_VIRTUAL";
      publish_zero_desired(debug);
      return;
    }

    debug.state = "TRACKING_VIRTUAL";
    debug.desired_yaw_cmd = std::clamp(debug.yaw_error, -1.0, 1.0);
    const double forward_scale = std::max(0.0, std::cos(debug.yaw_error));
    debug.desired_forward_cmd = std::min(0.5, debug.distance) * forward_scale;
    publish_debug(debug);
    publish_desired(debug.desired_forward_cmd, debug.desired_yaw_cmd);
  }

  void publish_zero_desired(const DebugValues & debug)
  {
    publish_debug(debug);
    publish_desired(0.0, 0.0);
  }

  void publish_desired(double forward, double yaw)
  {
    geometry_msgs::msg::Twist desired_message;
    desired_message.linear.x = forward;
    desired_message.angular.z = yaw;
    desired_command_publisher_->publish(desired_message);
  }

  void publish_debug(const DebugValues & debug)
  {
    // These values are deliberately fixed at zero in the non-actuating validation phase.
    constexpr double forward_cmd = 0.0;
    constexpr double yaw_cmd = 0.0;

    std::ostringstream json;
    json << std::fixed << std::setprecision(6)
         << "{\"state\":\"" << debug.state
         << "\",\"path_frame\":\"" << debug.path_frame
         << "\",\"robot_map_x\":" << debug.robot_map_x
         << ",\"robot_map_y\":" << debug.robot_map_y
         << ",\"robot_map_yaw\":" << debug.robot_map_yaw
         << ",\"nearest_index\":" << debug.nearest_index
         << ",\"target_index\":" << debug.target_index
         << ",\"target_x\":" << debug.target_x
         << ",\"target_y\":" << debug.target_y
         << ",\"target_local_x\":" << debug.target_local_x
         << ",\"target_local_y\":" << debug.target_local_y
         << ",\"distance\":" << debug.distance
         << ",\"yaw_error\":" << debug.yaw_error
         << ",\"desired_forward_cmd\":" << debug.desired_forward_cmd
         << ",\"desired_yaw_cmd\":" << debug.desired_yaw_cmd
         << ",\"forward_cmd\":" << forward_cmd
         << ",\"yaw_cmd\":" << yaw_cmd
         << "}";

    std_msgs::msg::String debug_message;
    debug_message.data = json.str();
    debug_publisher_->publish(debug_message);
  }

  double lookahead_distance_ = 0.4;
  std::optional<nav_msgs::msg::Path> last_path_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<rclcpp::Time> last_odom_receipt_time_;
  std::string last_dog_state_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desired_command_publisher_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JieDogController>());
  rclcpp::shutdown();
  return 0;
}
