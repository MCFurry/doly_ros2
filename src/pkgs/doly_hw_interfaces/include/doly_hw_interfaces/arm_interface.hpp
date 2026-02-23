#pragma once
#include <ArmControl.h>
#include <ArmEvent.h>
#include <Helper.h>

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace arm_interface
{

class ArmInterface : public rclcpp::Node
{
public:
  explicit ArmInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onArmErrorStatic(uint16_t id, ArmSide side, ArmErrorType errorType)
  {
    if (instance_) {
      instance_->onDriveError(id, side, errorType);
    }
  }

  static void onArmStateChangeStatic(ArmSide side, ArmState state)
  {
    if (instance_) {
      instance_->onDriveStateChange(side, state);
    }
  }

private:
  void onDriveError(uint16_t id, ArmSide side, ArmErrorType errorType)
  {
    logger_.error("Arm error id={} side={} type={}", id, (int)side, (int)errorType);
  }

  void onDriveStateChange(ArmSide side, ArmState state)
  {
    logger_.info("Arm state side={} state={}", (int)side, (int)state);
  }

  void stateTimerCb();

  // Singleton instance for static callbacks
  static ArmInterface * instance_;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr left_arm_subscriber_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr right_arm_subscriber_;

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr left_arm_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr right_arm_state_publisher_;

  rclcpp::TimerBase::SharedPtr state_pub_timer_;

  uint16_t arm_max_angle_ = 0;

  ros2_fmt_logger::Logger logger_;
};

}  // namespace arm_interface
