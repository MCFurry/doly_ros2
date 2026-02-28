#include "doly_hw_interfaces/tof_interface.hpp"

#include <chrono>
#include <thread>

namespace tof_interface
{
TofInterface * TofInterface::instance_ = nullptr;

TofInterface::TofInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("tof_interface", options), logger_(this->get_logger())
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // get TofControl version
  logger_.info("TofControl Version:{:.3f}", TofControl::getVersion());

  // Initialize ToF Control
  if (TofControl::init() < 0) {
    logger_.error("TofControl init failed");
    return;
  }

  // Register event listeners
  TofEvent::AddListenerOnProximityGesture(&TofInterface::onProximityGestureStatic);
  TofEvent::AddListenerOnProximityThreshold(&TofInterface::onProximityThresholdStatic);

  // Create publishers
  left_tof_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("tof/left", 10);
  right_tof_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("tof/right", 10);

  // Create timer at configurable rate
  const double freq = this->declare_parameter("freq", 10.0);
  publish_timer_ = create_timer(
    std::chrono::duration<double>{1.0 / freq}, [this] { publishTimerCb(); });

  logger_.info("TofInterface has been started.");
}

void TofInterface::publishTimerCb()
{
  std::vector<TofData> sensors = TofControl::getSensorsData();
  if (sensors.size() < 2) {
    logger_.warn("Failed to read ToF sensor data");
    return;
  }

  for (const auto & data : sensors) {
    auto msg = makeRangeMsg(
      data.side == TofSide::LEFT ? "tof_left" : "tof_right", data.range_mm);

    if (data.side == TofSide::LEFT) {
      left_tof_publisher_->publish(msg);
    } else {
      right_tof_publisher_->publish(msg);
    }
  }
}

sensor_msgs::msg::Range TofInterface::makeRangeMsg(const std::string & frame_id, int range_mm)
{
  sensor_msgs::msg::Range msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = frame_id;
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.field_of_view = 0.443;  // ~25 degrees in radians (VL6180 typical FoV)
  msg.min_range = 0.0;
  msg.max_range = 0.255;  // VL6180 max range ~255 mm
  msg.range = static_cast<float>(range_mm) / 1000.0f;  // convert mm to meters
  return msg;
}

}  // namespace tof_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  tof_interface::TofInterface tof_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(tof_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  TofEvent::RemoveListenerOnProximityGesture(&tof_interface::TofInterface::onProximityGestureStatic);
  TofEvent::RemoveListenerOnProximityThreshold(
    &tof_interface::TofInterface::onProximityThresholdStatic);
  TofControl::dispose();

  return EXIT_SUCCESS;
}
