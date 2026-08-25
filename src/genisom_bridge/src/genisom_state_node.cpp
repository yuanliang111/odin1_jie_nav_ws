#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "zsibot_sdk/zsibot_api.h"

namespace
{
const char * control_mode_name(zsibot::ControlMode mode)
{
  switch (mode) {
    case zsibot::ControlMode::CM_STAND_UP:
      return "stand_up";
    case zsibot::ControlMode::CM_SIT_DOWN:
      return "sit_down";
    case zsibot::ControlMode::CM_LOCK_MODE:
      return "lock";
    case zsibot::ControlMode::CM_EMERGENCY_STOP:
      return "emergency_stop";
    case zsibot::ControlMode::CM_MOVE_MODE:
      return "move";
    case zsibot::ControlMode::CM_BALANCE_STAND_MODE:
      return "balance_stand";
    case zsibot::ControlMode::CM_INIT:
      return "initializing";
    case zsibot::ControlMode::CM_NULL:
    default:
      return "unknown";
  }
}

const char * motion_mode_name(zsibot::MotionMode mode)
{
  switch (mode) {
    case zsibot::MotionMode::MM_RUNNING:
      return "running";
    case zsibot::MotionMode::MM_REST:
      return "rest";
    case zsibot::MotionMode::MM_FORBID:
      return "forbidden";
    case zsibot::MotionMode::MM_NULL:
    default:
      return "unknown";
  }
}
}  // namespace

class GenisomStateNode : public rclcpp::Node
{
public:
  GenisomStateNode()
  : Node("genisom_state_node")
  {
    const auto robot_ip = declare_parameter<std::string>("robot_ip", "192.168.234.1");
    const auto send_port = declare_parameter<int>("send_port", 8081);
    const auto recv_port = declare_parameter<int>("recv_port", 8080);

    if (send_port <= 0 || send_port > 65535 || recv_port <= 0 || recv_port > 65535) {
      throw std::invalid_argument("GENISOM UDP ports must be in the range 1..65535");
    }

    // Construction starts the vendor SDK's receive and heartbeat threads only. This node never
    // requests control and never sends a motion input.
    executor_ = std::make_unique<zsibot::ZsibotExecutor>(
      zsibot::Role::ROLE_REMOTE,
      robot_ip,
      static_cast<uint32_t>(send_port),
      static_cast<uint32_t>(recv_port));

    publisher_ = create_publisher<std_msgs::msg::String>("/dog/state", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&GenisomStateNode::publish_state, this));
  }

private:
  void publish_state()
  {
    const auto position = executor_->GetPosition();
    const auto velocity = executor_->GetWorldVelocity();
    const auto control_mode = executor_->GetControlMode();
    const auto motion_mode = executor_->GetMotionMode();

    std::ostringstream json;
    json << std::boolalpha
         << "{\"connected\":" << executor_->IsConnected()
         << ",\"x\":" << position[0]
         << ",\"y\":" << position[1]
         << ",\"z\":" << position[2]
         << ",\"vx\":" << velocity[0]
         << ",\"vy\":" << velocity[1]
         << ",\"vz\":" << velocity[2]
         << ",\"control_mode\":\"" << control_mode_name(control_mode) << "\""
         << ",\"motion_mode\":\"" << motion_mode_name(motion_mode) << "\""
         << ",\"power_percent\":" << executor_->GetPower()
         << "}";

    std_msgs::msg::String message;
    message.data = json.str();
    publisher_->publish(message);
  }

  std::unique_ptr<zsibot::ZsibotExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenisomStateNode>());
  rclcpp::shutdown();
  return 0;
}
