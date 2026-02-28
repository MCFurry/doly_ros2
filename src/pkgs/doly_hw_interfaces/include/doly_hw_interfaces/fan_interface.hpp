#pragma once
#include <FanControl.h>
#include <Helper.h>

#include <cstdint>
#include <doly_msgs/srv/set_fan_speed.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

namespace fan_interface
{

  constexpr int8_t AUTO_FAN_SPEED = -1;  // Special value to indicate automatic fan control
  constexpr int8_t MAX_FAN_SPEED = 100;  // Maximum fan speed percentage

class FanInterface : public rclcpp::Node
{
public:
  explicit FanInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void setFanSpeedCallback(
    const std::shared_ptr<doly_msgs::srv::SetFanSpeed::Request> request,
    std::shared_ptr<doly_msgs::srv::SetFanSpeed::Response> response);

  ros2_fmt_logger::Logger logger_;

  rclcpp::Service<doly_msgs::srv::SetFanSpeed>::SharedPtr set_fan_speed_service_;

  int8_t current_fan_speed_ = AUTO_FAN_SPEED;  // -1 for auto, 0-100 for manual
};

}  // namespace fan_interface
