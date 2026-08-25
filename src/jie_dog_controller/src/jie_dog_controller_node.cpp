#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class JieDogController : public rclcpp::Node
{
public:
  JieDogController()
  : Node("jie_dog_controller")
  {
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&JieDogController::path_callback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<std_msgs::msg::String>(
      "/dog/state", 10,
      std::bind(&JieDogController::state_callback, this, std::placeholders::_1));
    debug_publisher_ = create_publisher<std_msgs::msg::String>("/dog/control_debug", 10);
  }

private:
  void state_callback(const std_msgs::msg::String::SharedPtr message)
  {
    last_dog_state_ = message->data;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr message)
  {
    last_path_ = *message;

    for (const auto & pose_stamped : last_path_.poses) {
      const auto & target = pose_stamped.pose.position;
      if (!std::isfinite(target.x) || !std::isfinite(target.y)) {
        continue;
      }

      const double distance = std::hypot(target.x, target.y);
      const double yaw_error = std::atan2(target.y, target.x);
      const double desired_forward_cmd = std::min(0.5, distance);
      const double desired_yaw_cmd = std::clamp(yaw_error, -1.0, 1.0);
      publish_debug(
        "TRACKING", target.x, target.y, distance, yaw_error, desired_forward_cmd, desired_yaw_cmd);
      RCLCPP_INFO(
        get_logger(),
        "TRACKING target=(%.3f, %.3f), distance=%.3f, yaw_error=%.3f, desired=(%.3f, %.3f)",
        target.x, target.y, distance, yaw_error, desired_forward_cmd, desired_yaw_cmd);
      return;
    }

    publish_debug("NO_VALID_TARGET", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    RCLCPP_WARN(get_logger(), "Received planned_path without a finite XY target");
  }

  void publish_debug(
    const std::string & state, double target_x, double target_y, double distance, double yaw_error,
    double desired_forward_cmd, double desired_yaw_cmd)
  {
    // These values are deliberately fixed at zero in the non-actuating validation phase.
    constexpr double forward_cmd = 0.0;
    constexpr double yaw_cmd = 0.0;

    std::ostringstream json;
    json << std::fixed << std::setprecision(6)
         << "{\"state\":\"" << state
         << "\",\"target_x\":" << target_x
         << ",\"target_y\":" << target_y
         << ",\"distance\":" << distance
         << ",\"yaw_error\":" << yaw_error
         << ",\"desired_forward_cmd\":" << desired_forward_cmd
         << ",\"desired_yaw_cmd\":" << desired_yaw_cmd
         << ",\"forward_cmd\":" << forward_cmd
         << ",\"yaw_cmd\":" << yaw_cmd
         << "}";

    std_msgs::msg::String debug_message;
    debug_message.data = json.str();
    debug_publisher_->publish(debug_message);
  }

  nav_msgs::msg::Path last_path_;
  std::string last_dog_state_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JieDogController>());
  rclcpp::shutdown();
  return 0;
}
