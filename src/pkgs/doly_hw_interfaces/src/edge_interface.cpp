#include "doly_hw_interfaces/edge_interface.hpp"

#include <chrono>
#include <thread>

namespace edge_interface
{
EdgeInterface * EdgeInterface::instance_ = nullptr;

EdgeInterface::EdgeInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("edge_interface", options), logger_(this->get_logger())
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // get EdgeControl version
  logger_.info("EdgeControl Version:{:.3f}", EdgeControl::getVersion());

  // Initialize EdgeControl
  if (EdgeControl::init() < 0) {
    logger_.error("EdgeControl init failed");
    return;
  }

  // Register event listeners
  EdgeEvent::AddListenerOnChange(&EdgeInterface::onEdgeChangeStatic);
  EdgeEvent::AddListenerOnGapDetect(&EdgeInterface::onGapDetectStatic);

  // Create transient_local publishers so late-joining subscribers get the last state
  rclcpp::QoS qos(1);
  qos.transient_local();

  const std::map<SensorId, std::string> sensor_topics = {
    {SensorId::FRONT_LEFT,  "edge/front_left"},
    {SensorId::FRONT_RIGHT, "edge/front_right"},
    {SensorId::BACK_LEFT,   "edge/back_left"},
    {SensorId::BACK_RIGHT,  "edge/back_right"},
  };

  for (const auto & [id, topic] : sensor_topics) {
    edge_publishers_[id] = this->create_publisher<std_msgs::msg::Bool>(topic, qos);
    publishSensorState(id, true);  // initial state: ground detected
  }

  logger_.info("EdgeInterface has been started.");
}

void EdgeInterface::onEdgeChange(std::vector<IrSensor> sensors)
{
  for (const auto & sensor : sensors) {
    bool ground_detected = (sensor.state == GpioState::HIGH);
    logger_.debug(
      "Edge sensor id={} ground={}", static_cast<int>(sensor.id), ground_detected);
    publishSensorState(sensor.id, ground_detected);
  }
}

void EdgeInterface::onGapDetect(GapDirection gap_type)
{
  logger_.warn("Gap detected: direction={}", static_cast<int>(gap_type));
}

void EdgeInterface::publishSensorState(SensorId id, bool ground_detected)
{
  std_msgs::msg::Bool msg;
  msg.data = ground_detected;
  edge_publishers_.at(id)->publish(msg);
}

}  // namespace edge_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  edge_interface::EdgeInterface edge_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(edge_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  EdgeEvent::RemoveListenerOnChange(&edge_interface::EdgeInterface::onEdgeChangeStatic);
  EdgeEvent::RemoveListenerOnGapDetect(&edge_interface::EdgeInterface::onGapDetectStatic);
  EdgeControl::dispose();

  return EXIT_SUCCESS;
}
