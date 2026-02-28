#include "doly_hw_interfaces/touch_interface.hpp"

namespace touch_interface
{
TouchInterface * TouchInterface::instance_ = nullptr;

TouchInterface::TouchInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("touch_interface", options), logger_(this->get_logger())
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // get TouchControl version
  logger_.info("TouchControl Version:{:.3f}", TouchControl::getVersion());

  // Initialize TouchControl
  if (TouchControl::init() < 0) {
    logger_.error("TouchControl init failed");
    return;
  }

  touch_event_publisher_ = this->create_publisher<doly_msgs::msg::TouchEvent>("touch/event", 10);

  // Subscribe only to touch events (ignore touch activity)
  TouchEvent::AddListenerOnTouch(&TouchInterface::onTouchStatic);

  logger_.info("TouchInterface has been started.");
}

void TouchInterface::onTouch(TouchSide side, TouchState state)
{
  doly_msgs::msg::TouchEvent msg;
  msg.side = static_cast<uint8_t>(side);
  msg.state = static_cast<uint8_t>(state);
  touch_event_publisher_->publish(msg);

  logger_.debug("Touch event side={} state={}", msg.side, msg.state);
}

}  // namespace touch_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  touch_interface::TouchInterface touch_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(touch_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  TouchEvent::RemoveListenerOnTouch(&touch_interface::TouchInterface::onTouchStatic);
  TouchControl::dispose();

  return EXIT_SUCCESS;
}
