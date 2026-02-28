#pragma once
#include <EdgeControl.h>
#include <EdgeEvent.h>
#include <EdgeEventListener.h>
#include <Helper.h>

#include <cstdint>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <std_msgs/msg/bool.hpp>

namespace edge_interface
{

class EdgeInterface : public rclcpp::Node
{
public:
  explicit EdgeInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onEdgeChangeStatic(std::vector<IrSensor> sensors)
  {
    if (instance_) {
      instance_->onEdgeChange(sensors);
    }
  }

  static void onGapDetectStatic(GapDirection gap_type)
  {
    if (instance_) {
      instance_->onGapDetect(gap_type);
    }
  }

private:
  void onEdgeChange(std::vector<IrSensor> sensors);
  void onGapDetect(GapDirection gap_type);

  void publishSensorState(SensorId id, bool ground_detected);

  ros2_fmt_logger::Logger logger_;

  std::map<SensorId, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> edge_publishers_;

  // Singleton instance for static callbacks
  static EdgeInterface * instance_;
};

}  // namespace edge_interface
