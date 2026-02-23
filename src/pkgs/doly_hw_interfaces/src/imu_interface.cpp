#include "doly_hw_interfaces/imu_interface.hpp"

#include <chrono>
#include <thread>

namespace imu_interface
{
ImuInterface * ImuInterface::instance_ = nullptr;

ImuInterface::ImuInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_interface", options),
  logger_(this->get_logger()),
  imu_publisher_(this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10)),
  state_pub_timer_{create_timer(
    std::chrono::duration<double>{1.0 / this->declare_parameter("freq", 10.0)},
    [this] { stateTimerCb(); })}
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // For better performance provide actual calibrated IMU offsets or calculate them once after initialization
  // with the help of ImuControl::calculate_offsets() and use them for next initializations.
  // in this example we will use Helper to get previously saved offsets

  // Read settings
  // one time read is sufficient for the lifetime of the application
  int8_t res = Helper::readSettings();
  if (res < 0) {
    logger_.error("Read settings failed with code: {}", res);
    return;
  }

  // Get pre defined IMU offsets
  int16_t gx, gy, gz, ax, ay, az;
  res = Helper::getImuOffsets(gx, gy, gz, ax, ay, az);
  if (res < 0) {
    logger_.error("Get IMU offsets failed with code: {}", res);
    return;
  }

  // Initialize IMU Control with offsets
  // delay 1 second before processing events
  if (ImuControl::init(1, gx, gy, gz, ax, ay, az) < 0) {
    logger_.error("ImuControl init failed");
    return;
  }

  // get ImuControl version
  logger_.info("ImuControl Version:{:.3f}", ImuControl::getVersion());

  // Init frame-id
  current_imu_data_.header.frame_id = this->declare_parameter<std::string>("frame_id", "imu");

  // Register event listeners
  ImuEvent::AddListenerUpdateEvent(&ImuInterface::onImuUpdateStatic);
  ImuEvent::AddListenerGestureEvent(&ImuInterface::onImuGestureStatic);
}

void ImuInterface::stateTimerCb() { imu_publisher_->publish(current_imu_data_); }

}  // namespace imu_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  imu_interface::ImuInterface imu_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(imu_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  ImuEvent::RemoveListenerUpdateEvent(&imu_interface::ImuInterface::onImuUpdateStatic);
  ImuEvent::RemoveListenerGestureEvent(&imu_interface::ImuInterface::onImuGestureStatic);
  ImuControl::dispose();

  return EXIT_SUCCESS;
}