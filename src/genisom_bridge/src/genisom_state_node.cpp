#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "zsibot_sdk/zsibot_api.h"

namespace
{
enum class ActuationState
{
  DISABLED,
  CONNECTING,
  WAITING_CONTROL,
  WAITING_STATE,
  READY_ZERO,
  ACTIVE,
  STALE_COMMAND,
  FAULT,
};

const char * actuation_state_name(ActuationState state)
{
  switch (state) {
    case ActuationState::DISABLED:
      return "DISABLED";
    case ActuationState::CONNECTING:
      return "CONNECTING";
    case ActuationState::WAITING_CONTROL:
      return "WAITING_CONTROL";
    case ActuationState::WAITING_STATE:
      return "WAITING_STATE";
    case ActuationState::READY_ZERO:
      return "READY_ZERO";
    case ActuationState::ACTIVE:
      return "ACTIVE";
    case ActuationState::STALE_COMMAND:
      return "STALE_COMMAND";
    case ActuationState::FAULT:
    default:
      return "FAULT";
  }
}

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

bool normal_locomotion_motion_state(zsibot::MotionMode mode)
{
  // MotionMode represents laboratory/stunt activity, not ordinary locomotion permission.
  // MM_FORBID means no laboratory mode and MM_REST means no stunt is in progress; both
  // are normal states for navigation. Only an active stunt or an invalid state blocks it.
  switch (mode) {
    case zsibot::MotionMode::MM_FORBID:
    case zsibot::MotionMode::MM_REST:
      return true;
    case zsibot::MotionMode::MM_RUNNING:
    case zsibot::MotionMode::MM_NULL:
    default:
      return false;
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
    actuation_capable_ = declare_parameter<bool>("actuation_capable", false);
    max_forward_normalized_ = declare_parameter<double>("max_forward_normalized", 0.0);
    max_yaw_normalized_ = declare_parameter<double>("max_yaw_normalized", 0.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.3);

    if (send_port <= 0 || send_port > 65535 || recv_port <= 0 || recv_port > 65535) {
      throw std::invalid_argument("GENISOM UDP ports must be in the range 1..65535");
    }
    if (max_forward_normalized_ < 0.0 || max_forward_normalized_ > 1.0 ||
      max_yaw_normalized_ < 0.0 || max_yaw_normalized_ > 1.0)
    {
      throw std::invalid_argument("GENISOM normalized command limits must be in the range 0..1");
    }
    if (command_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("command_timeout_sec must be positive");
    }

    // This is the only ZsibotExecutor in the workspace. The default role is read-only;
    // Role::ROLE_SDK is selected only by the explicit actuation_capable startup parameter.
    const auto role = actuation_capable_ ? zsibot::Role::ROLE_SDK : zsibot::Role::ROLE_REMOTE;
    executor_ = std::make_unique<zsibot::ZsibotExecutor>(
      role, robot_ip, static_cast<uint32_t>(send_port), static_cast<uint32_t>(recv_port));

    state_publisher_ = create_publisher<std_msgs::msg::String>("/dog/state", 10);
    actuation_debug_publisher_ = create_publisher<std_msgs::msg::String>("/dog/actuation_debug", 10);
    desired_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/dog/desired_cmd_normalized", 10,
      std::bind(&GenisomStateNode::desired_command_callback, this, std::placeholders::_1));
    actuation_service_ = create_service<std_srvs::srv::SetBool>(
      "/dog/set_actuation_enabled",
      std::bind(&GenisomStateNode::set_actuation_enabled, this, std::placeholders::_1,
        std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&GenisomStateNode::update, this));
  }

  ~GenisomStateNode() override
  {
    // Shutdown sends only zero input, and only after SDK control was confirmed.
    if (executor_ && sdk_control_confirmed_ && executor_->IsConnected()) {
      send_zero_commands(3);
    }
  }

private:
  struct ActuationDebug
  {
    ActuationState state = ActuationState::DISABLED;
    bool connected = false;
    bool command_fresh = false;
    double desired_forward = 0.0;
    double desired_yaw = 0.0;
    double limited_forward = 0.0;
    double limited_yaw = 0.0;
    zsibot::ControlMode control_mode = zsibot::ControlMode::CM_NULL;
    zsibot::MotionMode motion_mode = zsibot::MotionMode::MM_NULL;
  };

  void desired_command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    latest_desired_command_ = *message;
    // Receipt time uses the local ROS clock and intentionally ignores any external timestamp.
    last_command_receipt_time_ = now();
  }

  void set_actuation_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!request->data) {
      const bool was_sdk_control_confirmed = sdk_control_confirmed_;
      send_zero_commands(3);
      if (was_sdk_control_confirmed) {
        executor_->SetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      }
      actuation_enabled_ = false;
      sdk_control_confirmed_ = false;
      fault_latched_ = false;
      latest_desired_command_.reset();
      last_command_receipt_time_.reset();
      actuation_state_ = ActuationState::DISABLED;
      response->success = true;
      response->message = "Actuation disabled; zero input sent and confirmed SDK control returned to remote.";
      return;
    }

    if (!actuation_capable_) {
      response->success = false;
      response->message = "Actuation is unavailable because actuation_capable is false.";
      return;
    }

    if (actuation_enabled_) {
      response->success = true;
      response->message = "Actuation enable request is already active; waiting for SDK confirmation.";
      return;
    }

    // Only this explicit human service action requests SDK control. Desired commands cannot.
    actuation_enabled_ = true;
    sdk_control_confirmed_ = false;
    fault_latched_ = false;
    latest_desired_command_.reset();
    last_command_receipt_time_.reset();
    actuation_state_ = ActuationState::CONNECTING;
    executor_->SetCmd(zsibot::CmdCode::CMD_SDK_CONTROL_RIGHT);
    response->success = true;
    response->message = "SDK control requested; waiting for connected SDK-mode confirmation.";
  }

  void update()
  {
    publish_state();

    ActuationDebug debug;
    debug.connected = executor_->IsConnected();
    debug.control_mode = executor_->GetControlMode();
    debug.motion_mode = executor_->GetMotionMode();
    debug.desired_forward = desired_forward();
    debug.desired_yaw = desired_yaw();
    debug.command_fresh = command_is_fresh();

    if (!actuation_enabled_) {
      actuation_state_ = ActuationState::DISABLED;
      debug.state = actuation_state_;
      publish_actuation_debug(debug);
      return;
    }

    if (fault_latched_) {
      actuation_state_ = ActuationState::FAULT;
      send_zero_commands(1);
      debug.state = actuation_state_;
      publish_actuation_debug(debug);
      return;
    }

    if (!debug.connected) {
      actuation_state_ = ActuationState::CONNECTING;
      debug.state = actuation_state_;
      publish_actuation_debug(debug);
      return;
    }

    if (!sdk_control_confirmed_) {
      if (executor_->GetFunctionMode() == zsibot::FunctionMode::FM_SDK) {
        sdk_control_confirmed_ = true;
        actuation_state_ = ActuationState::READY_ZERO;
        send_zero_commands(1);
      } else {
        actuation_state_ = ActuationState::WAITING_CONTROL;
      }
      debug.state = actuation_state_;
      publish_actuation_debug(debug);
      return;
    }

    if (executor_->GetFunctionMode() != zsibot::FunctionMode::FM_SDK ||
      debug.control_mode != zsibot::ControlMode::CM_MOVE_MODE ||
      !executor_->GetFaultInfo().empty())
    {
      fault_latched_ = true;
      actuation_state_ = ActuationState::FAULT;
      send_zero_commands(3);
      debug.state = actuation_state_;
      publish_actuation_debug(debug);
      return;
    }

    if (!normal_locomotion_motion_state(debug.motion_mode)) {
      actuation_state_ = ActuationState::WAITING_STATE;
      send_zero_commands(1);
      debug.state = actuation_state_;
      publish_actuation_debug(debug);
      return;
    }

    if (!debug.command_fresh) {
      actuation_state_ = ActuationState::STALE_COMMAND;
      send_zero_commands(1);
      debug.state = actuation_state_;
      publish_actuation_debug(debug);
      return;
    }

    debug.limited_forward = std::clamp(
      debug.desired_forward, -max_forward_normalized_, max_forward_normalized_);
    debug.limited_yaw = std::clamp(
      debug.desired_yaw, -max_yaw_normalized_, max_yaw_normalized_);
    actuation_state_ = ActuationState::ACTIVE;
    send_remote(debug.limited_forward, debug.limited_yaw);
    debug.state = actuation_state_;
    publish_actuation_debug(debug);
  }

  bool command_is_fresh() const
  {
    return latest_desired_command_ && last_command_receipt_time_ &&
           (now() - *last_command_receipt_time_) <=
             rclcpp::Duration::from_seconds(command_timeout_sec_);
  }

  double desired_forward() const
  {
    return latest_desired_command_ ? latest_desired_command_->linear.x : 0.0;
  }

  double desired_yaw() const
  {
    return latest_desired_command_ ? latest_desired_command_->angular.z : 0.0;
  }

  void send_zero_commands(std::size_t count)
  {
    if (!executor_ || !sdk_control_confirmed_) {
      return;
    }
    for (std::size_t index = 0; index < count; ++index) {
      send_remote(0.0, 0.0);
    }
  }

  void send_remote(double forward, double yaw)
  {
    // The unused joystick axes and all fourteen buttons remain fixed at zero.
    const std::array<zsibot::float32_t, 4> joystick{
      static_cast<zsibot::float32_t>(forward), static_cast<zsibot::float32_t>(yaw), 0.0F, 0.0F};
    const std::array<zsibot::float32_t, 14> buttons{};
    executor_->SetRemote(joystick, buttons);
  }

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
    state_publisher_->publish(message);
  }

  void publish_actuation_debug(const ActuationDebug & debug)
  {
    std::ostringstream json;
    json << std::boolalpha
         << "{\"state\":\"" << actuation_state_name(debug.state) << "\""
         << ",\"connected\":" << debug.connected
         << ",\"actuation_capable\":" << actuation_capable_
         << ",\"actuation_enabled\":" << actuation_enabled_
         << ",\"sdk_control_confirmed\":" << sdk_control_confirmed_
         << ",\"command_fresh\":" << debug.command_fresh
         << ",\"desired_forward\":" << debug.desired_forward
         << ",\"desired_yaw\":" << debug.desired_yaw
         << ",\"limited_forward\":" << debug.limited_forward
         << ",\"limited_yaw\":" << debug.limited_yaw
         << ",\"max_forward_normalized\":" << max_forward_normalized_
         << ",\"max_yaw_normalized\":" << max_yaw_normalized_
         << ",\"control_mode\":\"" << control_mode_name(debug.control_mode) << "\""
         << ",\"motion_mode\":\"" << motion_mode_name(debug.motion_mode) << "\""
         << "}";

    std_msgs::msg::String message;
    message.data = json.str();
    actuation_debug_publisher_->publish(message);
  }

  bool actuation_capable_ = false;
  bool actuation_enabled_ = false;
  bool sdk_control_confirmed_ = false;
  bool fault_latched_ = false;
  double max_forward_normalized_ = 0.0;
  double max_yaw_normalized_ = 0.0;
  double command_timeout_sec_ = 0.3;
  ActuationState actuation_state_ = ActuationState::DISABLED;
  std::optional<geometry_msgs::msg::Twist> latest_desired_command_;
  std::optional<rclcpp::Time> last_command_receipt_time_;
  std::unique_ptr<zsibot::ZsibotExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr actuation_debug_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_command_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr actuation_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenisomStateNode>());
  rclcpp::shutdown();
  return 0;
}
