#include "doly_hw_interfaces/led_interface.hpp"

#include <chrono>
#include <thread>

namespace led_interface
{
LedInterface * LedInterface::instance_ = nullptr;

LedInterface::LedInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("led_interface", options), logger_(this->get_logger())
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // get LedControl version
  logger_.info("LedControl Version:{:.3f}", LedControl::getVersion());

  // Register event listeners
  LedEvent::AddListenerOnComplete(&LedInterface::onLedCompleteStatic);
  LedEvent::AddListenerOnError(&LedInterface::onLedErrorStatic);

  // Initialize LedControl
  if (LedControl::init() < 0) {
    logger_.error("LedControl init failed");
    return;
  }

  left_led_subscriber_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "led/left", 10,
    [this](const std_msgs::msg::ColorRGBA::SharedPtr msg) { this->leftColorCallback(msg); });

  right_led_subscriber_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "led/right", 10,
    [this](const std_msgs::msg::ColorRGBA::SharedPtr msg) { this->rightColorCallback(msg); });

  logger_.info("LedInterface has been started.");
}

void LedInterface::leftColorCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
{
  setLedColor(LedSide::LEFT, msg);
}

void LedInterface::rightColorCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
{
  setLedColor(LedSide::RIGHT, msg);
}

void LedInterface::setLedColor(LedSide side, const std_msgs::msg::ColorRGBA::SharedPtr & msg)
{
  // ColorRGBA uses float 0.0-1.0, convert to uint8_t 0-255
  Color color = Color::getColor(
    static_cast<uint8_t>(std::clamp(msg->r, 0.0f, 1.0f) * 255.0f),
    static_cast<uint8_t>(std::clamp(msg->g, 0.0f, 1.0f) * 255.0f),
    static_cast<uint8_t>(std::clamp(msg->b, 0.0f, 1.0f) * 255.0f));

  LedActivity activity;
  activity.mainColor = color;
  activity.fadeColor = color;
  activity.fade_time = 0;  // instant

  LedControl::processActivity(side == LedSide::LEFT ? 0 : 2, side, activity);
}

}  // namespace led_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  led_interface::LedInterface led_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(led_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  LedEvent::RemoveListenerOnComplete(&led_interface::LedInterface::onLedCompleteStatic);
  LedEvent::RemoveListenerOnError(&led_interface::LedInterface::onLedErrorStatic);
  LedControl::dispose();

  return EXIT_SUCCESS;
}
