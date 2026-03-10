#pragma once
#include <EyeControl.h>
#include <EyeEvent.h>
#include <Helper.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <doly_msgs/action/eye_animation.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

namespace eye_interface
{

using EyeAnimation = doly_msgs::action::EyeAnimation;
using GoalHandleEyeAnimation = rclcpp_action::ServerGoalHandle<EyeAnimation>;

class EyeInterface : public rclcpp::Node
{
public:
  explicit EyeInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onEyeStartStatic(uint16_t id)
  {
    if (instance_) {
      instance_->onEyeStart(id);
    }
  }

  static void onEyeCompleteStatic(uint16_t id)
  {
    if (instance_) {
      instance_->onEyeComplete(id);
    }
  }

  static void onEyeAbortStatic(uint16_t id)
  {
    if (instance_) {
      instance_->onEyeAbort(id);
    }
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const EyeAnimation::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleEyeAnimation> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleEyeAnimation> goal_handle);

  void execute(const std::shared_ptr<GoalHandleEyeAnimation> goal_handle);

  void onEyeStart(uint16_t id);
  void onEyeComplete(uint16_t id);
  void onEyeAbort(uint16_t id);

  ros2_fmt_logger::Logger logger_;

  rclcpp_action::Server<EyeAnimation>::SharedPtr action_server_;

  std::mutex animation_mutex_;
  std::condition_variable animation_cv_;
  std::atomic<bool> animation_done_{false};
  std::atomic<bool> animation_success_{false};
  std::atomic<uint16_t> active_animation_id_{0};

  // Singleton instance for static callbacks
  static EyeInterface * instance_;
};

}  // namespace eye_interface
