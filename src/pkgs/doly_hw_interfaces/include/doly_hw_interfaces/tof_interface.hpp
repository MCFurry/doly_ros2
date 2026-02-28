#pragma once
#include <Helper.h>
#include <TofControl.h>
#include <TofEvent.h>
#include <TofEventListener.h>

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace tof_interface
{

class TofInterface : public rclcpp::Node
{
public:
  explicit TofInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onProximityGestureStatic(TofGesture left, TofGesture right)
  {
    if (instance_) {
      instance_->onProximityGesture(left, right);
    }
  }

  static void onProximityThresholdStatic(TofData left, TofData right)
  {
    if (instance_) {
      instance_->onProximityThreshold(left, right);
    }
  }

private:
  void onProximityGesture(TofGesture left, TofGesture right)
  {
    logger_.debug(
      "ToF gesture left={} right={}", static_cast<int>(left.type),
      static_cast<int>(right.type));
  }

  void onProximityThreshold(TofData left, TofData right)
  {
    logger_.debug(
      "ToF proximity left={}mm right={}mm", left.range_mm, right.range_mm);
  }

  void publishTimerCb();

  sensor_msgs::msg::Range makeRangeMsg(const std::string & frame_id, int range_mm);

  ros2_fmt_logger::Logger logger_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr left_tof_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr right_tof_publisher_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Singleton instance for static callbacks
  static TofInterface * instance_;
};

}  // namespace tof_interface
