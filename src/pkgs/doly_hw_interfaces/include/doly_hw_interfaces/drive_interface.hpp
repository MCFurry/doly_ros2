#pragma once
#include <DriveControl.h>
#include <DriveEvent.h>
#include <DriveEventListener.h>
#include <Helper.h>

#include <cstdint>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

constexpr double WHEEL_SEPARATION = 0.085;  // Distance between the wheels in meters
constexpr double MAX_WHEEL_SPEED = 0.1;     // Maximum wheel speed [m/s]

namespace drive_interface
{

class DriveInterface : public rclcpp::Node
{
public:
  explicit DriveInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onDriveErrorStatic(std::uint16_t id, DriveMotorSide side, DriveErrorType type)
  {
    if (instance_) {
      instance_->onDriveError(id, side, type);
    }
  }

  static void onDriveStateChangeStatic(DriveType driveType, DriveState state)
  {
    if (instance_) {
      instance_->onDriveStateChange(driveType, state);
    }
  }

private:
  void onDriveError(std::uint16_t id, DriveMotorSide side, DriveErrorType type)
  {
    logger_.error("Drive error id={} side={} type={}", id, (int)side, (int)type);
  }

  void onDriveStateChange(DriveType driveType, DriveState state)
  {
    logger_.debug("Drive state type={} state={}", (int)driveType, (int)state);
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber_;

  ros2_fmt_logger::Logger logger_;

  // Singleton instance for static callbacks
  static DriveInterface * instance_;
};

void twistToWheels(
  double v, double omega, double wheel_separation, double & left_speed, double & right_speed)
{
  left_speed = v - (wheel_separation / 2.0) * omega;
  right_speed = v + (wheel_separation / 2.0) * omega;
}

uint8_t speedToMotorCommand(double speed)
{
  // Map the speed to the range of motor commands (0-100)
  double scaled_speed = (speed / MAX_WHEEL_SPEED) * 100.0;
  scaled_speed = std::clamp(scaled_speed, -100.0, 100.0);
  return static_cast<uint8_t>(std::abs(scaled_speed));
}

}  // namespace drive_interface
