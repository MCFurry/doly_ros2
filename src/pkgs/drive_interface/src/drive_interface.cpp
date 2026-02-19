#include "drive_interface/drive_interface.hpp"

#include <chrono>
#include <thread>

namespace drive_interface
{
DriveInterface* DriveInterface::instance_ = nullptr;

DriveInterface::DriveInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("drive_interface", options), logger_(this->get_logger())
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // get DriveControl version
  logger_.info("DriveControl Version:{:.3f}", DriveControl::getVersion());

  // Register event listeners
  DriveEvent::AddListenerOnError(&DriveInterface::onDriveErrorStatic);
  DriveEvent::AddListenerOnStateChange(&DriveInterface::onDriveStateChangeStatic);

  // Initialize DriveControl
  if (DriveControl::init() != 0) {
    logger_.error("DriveControl init failed");
    return;
  }

  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->cmdVelCallback(msg); });
  logger_.info("DriveInterface has been started.");
}

void DriveInterface::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Convert cmd_vel to motor commands
  double speed_left, speed_right;
  twistToWheels(
    msg->twist.linear.x, msg->twist.angular.z, WHEEL_SEPARATION, speed_left, speed_right);
  DriveControl::freeDrive(speedToMotorCommand(speed_left), true, speed_left >= 0.0);
  DriveControl::freeDrive(speedToMotorCommand(speed_right), false, speed_right >= 0.0);
}

}  // namespace drive_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  drive_interface::DriveInterface drive_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(drive_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  DriveControl::dispose(true);  // dispose IMU as well
  DriveEvent::RemoveListenerOnError(&drive_interface::DriveInterface::onDriveErrorStatic);
  DriveEvent::RemoveListenerOnStateChange(&drive_interface::DriveInterface::onDriveStateChangeStatic);

  return EXIT_SUCCESS;
}