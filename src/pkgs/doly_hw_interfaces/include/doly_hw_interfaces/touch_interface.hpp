#pragma once
#include <Helper.h>
#include <TouchControl.h>
#include <TouchEvent.h>
#include <TouchEventListener.h>

#include <doly_msgs/msg/touch_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

namespace touch_interface
{

class TouchInterface : public rclcpp::Node
{
public:
  explicit TouchInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onTouchStatic(TouchSide side, TouchState state)
  {
    if (instance_) {
      instance_->onTouch(side, state);
    }
  }

private:
  void onTouch(TouchSide side, TouchState state);

  ros2_fmt_logger::Logger logger_;

  rclcpp::Publisher<doly_msgs::msg::TouchEvent>::SharedPtr touch_event_publisher_;

  // Singleton instance for static callbacks
  static TouchInterface * instance_;
};

}  // namespace touch_interface
