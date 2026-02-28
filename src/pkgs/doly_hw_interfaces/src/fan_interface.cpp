#include "doly_hw_interfaces/fan_interface.hpp"

namespace fan_interface
{

FanInterface::FanInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("fan_interface", options), logger_(this->get_logger())
{
  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // get FanControl version
  logger_.info("FanControl Version:{:.3f}", FanControl::getVersion());

  // Initialize FanControl with automatic temperature control enabled
  if (FanControl::init(true) < 0) {
    logger_.error("FanControl init failed");
    return;
  }

  // Create service for manual fan speed override
  set_fan_speed_service_ = this->create_service<doly_msgs::srv::SetFanSpeed>(
    "set_fan_speed",
    [this](
      const std::shared_ptr<doly_msgs::srv::SetFanSpeed::Request> request,
      std::shared_ptr<doly_msgs::srv::SetFanSpeed::Response> response) {
      this->setFanSpeedCallback(request, response);
    });

  logger_.info("FanInterface has been started (auto_control=true).");
}

void FanInterface::setFanSpeedCallback(
  const std::shared_ptr<doly_msgs::srv::SetFanSpeed::Request> request,
  std::shared_ptr<doly_msgs::srv::SetFanSpeed::Response> response)
{
  if (request->percentage > MAX_FAN_SPEED) {
    logger_.warn("FanSpeed percentage {} clamped to {}%", request->percentage, MAX_FAN_SPEED);
  } else if (request->percentage == AUTO_FAN_SPEED) {
    logger_.info("Setting fan to automatic mode");
    if (current_fan_speed_ >= 0) {
        FanControl::dispose();
        if (FanControl::init(true) < 0)
        {
            logger_.error("FanControl init failed");
            response->success = false;
            response->message = "FanControl init failed";
            return;
        }
    }
  } else if (request->percentage < AUTO_FAN_SPEED) {
    logger_.warn("FanSpeed percentage {} is invalid, must be between 0-100 or -1 for auto", request->percentage);
    response->success = false;
    response->message = "Invalid percentage value: " + std::to_string(request->percentage);
    return;
  }

  if (current_fan_speed_ < 0) {
    FanControl::dispose();
    if (FanControl::init(false) < 0)
	{
	  logger_.error("FanControl init failed");
      response->success = false;
      response->message = "FanControl init failed";
      return;
	}
  }

  int8_t res = FanControl::setFanSpeed(std::min(MAX_FAN_SPEED, request->percentage));
  if (res < 0) {
    response->success = false;
    response->message = "FanControl::setFanSpeed failed with code " + std::to_string(res);
    logger_.error("SetFanSpeed failed: code={}", res);
  } else {
    response->success = true;
    response->message = "Fan speed set to " + std::to_string(request->percentage) + "%";
    logger_.info("Fan speed set to {}%", request->percentage);
  }

  current_fan_speed_ = request->percentage;
}

}  // namespace fan_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  fan_interface::FanInterface fan_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(fan_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  FanControl::dispose();

  return EXIT_SUCCESS;
}
