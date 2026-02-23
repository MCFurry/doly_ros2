#include "doly_hw_interfaces/arm_interface.hpp"

#include <chrono>
#include <thread>

namespace arm_interface
{
ArmInterface * ArmInterface::instance_ = nullptr;

ArmInterface::ArmInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("arm_interface", options),
  logger_(this->get_logger()),
  left_arm_state_publisher_(this->create_publisher<std_msgs::msg::Float32>("left_arm_state", 1)),
  right_arm_state_publisher_(this->create_publisher<std_msgs::msg::Float32>("right_arm_state", 1)),
  state_pub_timer_{create_timer(
    std::chrono::duration<double>{1.0 / this->declare_parameter("freq", 10.0)},
    [this] { stateTimerCb(); })}
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // Read settings for servo calibration data, otherwise arm control won't work.
  // one time read is sufficient for the lifetime of the application
  int8_t res = Helper::readSettings();
  if (res < 0) {
    logger_.error("Read settings failed with code: {}", res);
    return;
  }

  // get ArmControl version
  logger_.info("ArmControl Version:{:.3f}", ArmControl::getVersion());

  // Register event listeners
  ArmEvent::AddListenerOnError(&ArmInterface::onArmErrorStatic);
  ArmEvent::AddListenerOnStateChange(&ArmInterface::onArmStateChangeStatic);

  // Initialize ArmControl
  if (ArmControl::init() < 0) {
    logger_.error("ArmControl init failed");
    return;
  }

  arm_max_angle_ = ArmControl::getMaxAngle();
  logger_.info("Arm max angle: {} rad", arm_max_angle_ * DEG2RADS);

  // Subscribers
  left_arm_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
    "left_arm_angle", 1, [this](const std_msgs::msg::Float32::SharedPtr msg) {
      uint16_t angle = std::min(static_cast<uint16_t>(msg->data / DEG2RADS), arm_max_angle_);
      ArmControl::setAngle(0, ArmSide::LEFT, 100, angle, false);
    });
  right_arm_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
    "right_arm_angle", 1, [this](const std_msgs::msg::Float32::SharedPtr msg) {
      uint16_t angle = std::min(static_cast<uint16_t>(msg->data / DEG2RADS), arm_max_angle_);
      ArmControl::setAngle(0, ArmSide::RIGHT, 100, angle, false);
    });
}

void ArmInterface::stateTimerCb()
{
  std_msgs::msg::Float32 left_state_msg;
  std_msgs::msg::Float32 right_state_msg;

  std::vector<ArmData> angles = ArmControl::getCurrentAngle(ArmSide::BOTH);

  left_state_msg.data =
    angles[0].side == ArmSide::LEFT ? angles[0].angle * DEG2RADS : angles[1].angle * DEG2RADS;
  right_state_msg.data =
    angles[0].side == ArmSide::RIGHT ? angles[0].angle * DEG2RADS : angles[1].angle * DEG2RADS;

  left_arm_state_publisher_->publish(left_state_msg);
  right_arm_state_publisher_->publish(right_state_msg);
}

}  // namespace arm_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  arm_interface::ArmInterface arm_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(arm_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  ArmEvent::RemoveListenerOnError(&arm_interface::ArmInterface::onArmErrorStatic);
  ArmEvent::RemoveListenerOnStateChange(&arm_interface::ArmInterface::onArmStateChangeStatic);
  ArmControl::dispose();

  return EXIT_SUCCESS;
}