#pragma once
#include <Helper.h>
#include <LedControl.h>
#include <LedEvent.h>
#include <LedEventListener.h>

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace led_interface
{

class LedInterface : public rclcpp::Node
{
public:
  explicit LedInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onLedCompleteStatic(uint16_t id, LedSide side)
  {
    if (instance_) {
      instance_->onLedComplete(id, side);
    }
  }

  static void onLedErrorStatic(uint16_t id, LedSide side, LedErrorType type)
  {
    if (instance_) {
      instance_->onLedError(id, side, type);
    }
  }

private:
  void onLedComplete(uint16_t id, LedSide side)
  {
    logger_.debug("Led complete id={} side={}", id, static_cast<int>(side));
  }

  void onLedError(uint16_t id, LedSide side, LedErrorType type)
  {
    logger_.error("Led error id={} side={} type={}", id, static_cast<int>(side), static_cast<int>(type));
  }

  void leftColorCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg);
  void rightColorCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg);

  void setLedColor(LedSide side, const std_msgs::msg::ColorRGBA::SharedPtr & msg);

  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr left_led_subscriber_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr right_led_subscriber_;

  ros2_fmt_logger::Logger logger_;

  // Singleton instance for static callbacks
  static LedInterface * instance_;
};

}  // namespace led_interface
