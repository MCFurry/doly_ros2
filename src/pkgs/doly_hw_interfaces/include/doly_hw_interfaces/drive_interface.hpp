#pragma once
#include <DriveControl.h>
#include <DriveEvent.h>
#include <DriveEventListener.h>
#include <Helper.h>
#include <ImuControl.h>

#include <cstdint>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

constexpr double WHEEL_SEPARATION = 0.085;  // Distance between the wheels in meters
constexpr double MAX_WHEEL_SPEED = 0.1;     // Maximum wheel speed [m/s]
constexpr float DEG2RADS = 0.0174533f;

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

  static void onImuUpdateStatic(ImuData data)
  {
    if (instance_) {
      instance_->onImuUpdate(data);
    }
  }

  static void onImuGestureStatic(ImuGesture type, GestureDirection from)
  {
    if (instance_) {
      instance_->onImuGesture(type, from);
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

  void onImuUpdate(ImuData data)
  {
    logger_.debug(
      "IMU Update - Yaw: {:.2f}, Pitch: {:.2f}, Roll: {:.2f}", data.ypr.yaw, data.ypr.pitch,
      data.ypr.roll);
    tf2::Quaternion explicit_quat;
    explicit_quat.setRPY(
      data.ypr.roll * DEG2RADS, data.ypr.pitch * DEG2RADS, data.ypr.yaw * DEG2RADS);
    current_imu_data_.header.stamp = this->get_clock()->now();
    current_imu_data_.orientation.x = explicit_quat.x();
    current_imu_data_.orientation.y = explicit_quat.y();
    current_imu_data_.orientation.z = explicit_quat.z();
    current_imu_data_.orientation.w = explicit_quat.w();

    current_imu_data_.linear_acceleration.x = data.linear_accel.x;
    current_imu_data_.linear_acceleration.y = data.linear_accel.y;
    current_imu_data_.linear_acceleration.z = data.linear_accel.z;
  }

  void onImuGesture(ImuGesture type, GestureDirection from)
  {
    logger_.debug(
      "IMU Gesture - Type: {}, Direction: {}", ImuEvent::getGestureStr(type),
      ImuEvent::getDirectionStr(from));
  }

  void imuTimerCb();

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr imu_pub_timer_;
  sensor_msgs::msg::Imu current_imu_data_;

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
