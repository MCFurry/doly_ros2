#pragma once
#include "ImuControl.h"
#include <Helper.h>

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace imu_interface
{

constexpr float DEG2RADS = 0.0174533f;

class ImuInterface : public rclcpp::Node
{
public:
  explicit ImuInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

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
  void onImuUpdate(ImuData data)
  {
    logger_.debug("IMU Update - Yaw: {:.2f}, Pitch: {:.2f}, Roll: {:.2f}", data.ypr.yaw, data.ypr.pitch, data.ypr.roll);
    tf2::Quaternion explicit_quat;
    explicit_quat.setRPY(data.ypr.roll * DEG2RADS, data.ypr.pitch * DEG2RADS, data.ypr.yaw * DEG2RADS);
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
    logger_.debug("IMU Gesture - Type: {}, Direction: {}", ImuEvent::getGestureStr(type), ImuEvent::getDirectionStr(from));
  }

  void stateTimerCb();

  ros2_fmt_logger::Logger logger_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  rclcpp::TimerBase::SharedPtr state_pub_timer_;

  sensor_msgs::msg::Imu current_imu_data_;

  // Singleton instance for static callbacks
  static ImuInterface * instance_;
};

}  // namespace imu_interface
