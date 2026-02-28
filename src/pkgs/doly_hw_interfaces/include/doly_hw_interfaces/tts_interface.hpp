#pragma once
#include <Helper.h>
#include <string>
#include <SoundControl.h>
#include <SoundEvent.h>
#include <TtsControl.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <doly_msgs/action/speak.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

namespace tts_interface
{

using Speak = doly_msgs::action::Speak;
using GoalHandleSpeak = rclcpp_action::ServerGoalHandle<Speak>;

constexpr uint16_t SOUND_ID = 42; // Arbitrary ID for TTS playback, since we only handle one at a time

class TtsInterface : public rclcpp::Node
{
public:
  explicit TtsInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  static void onSoundCompleteStatic(uint16_t id)
  {
    if (instance_) {
      instance_->onSoundComplete(id);
    }
  }

  static void onSoundErrorStatic(uint16_t id)
  {
    if (instance_) {
      instance_->onSoundError(id);
    }
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Speak::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleSpeak> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleSpeak> goal_handle);

  void execute(const std::shared_ptr<GoalHandleSpeak> goal_handle);

  void onSoundComplete(uint16_t id);
  void onSoundError(uint16_t id);

  ros2_fmt_logger::Logger logger_;

  rclcpp_action::Server<Speak>::SharedPtr action_server_;

  std::string tts_output_path_;

  // Synchronization for waiting on sound playback completion
  std::mutex playback_mutex_;
  std::condition_variable playback_cv_;
  std::atomic<bool> playback_done_{false};
  std::atomic<bool> playback_success_{false};

  // Singleton instance for static callbacks
  static TtsInterface * instance_;
};

}  // namespace tts_interface
