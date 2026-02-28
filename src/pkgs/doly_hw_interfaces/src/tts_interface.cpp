#include "doly_hw_interfaces/tts_interface.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <thread>

namespace tts_interface
{
TtsInterface * TtsInterface::instance_ = nullptr;

TtsInterface::TtsInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("tts_interface", options),
  logger_(this->get_logger()),
  tts_output_path_(this->declare_parameter<std::string>("tts_output_path", "/tmp/doly_tts.wav"))
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  // Initialize TTS with voice model
  logger_.info("TtsControl Version:{:.3f}", TtsControl::getVersion());
  logger_.info("Loading TTS voice model...");
  if (TtsControl::init(VoiceModel::MODEL_1, tts_output_path_) < 0) {
    logger_.error("TtsControl init failed");
    return;
  }
  logger_.info("TTS voice model loaded.");

  // Initialize SoundControl for playback
  if (SoundControl::init() < 0) {
    logger_.error("SoundControl init failed");
    return;
  }

  // Register sound event listeners
  SoundEvent::AddListenerOnComplete(&TtsInterface::onSoundCompleteStatic);
  SoundEvent::AddListenerOnError(&TtsInterface::onSoundErrorStatic);

  // Create action server
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<Speak>(
    this, "speak",
    std::bind(&TtsInterface::handleGoal, this, _1, _2),
    std::bind(&TtsInterface::handleCancel, this, _1),
    std::bind(&TtsInterface::handleAccepted, this, _1));

  logger_.info("TtsInterface has been started.");
}

rclcpp_action::GoalResponse TtsInterface::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Speak::Goal> goal)
{
  if (goal->text.empty()) {
    logger_.warn("Rejecting empty speech goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  logger_.info("Received speech goal: \"{}\"", goal->text);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TtsInterface::handleCancel(
  const std::shared_ptr<GoalHandleSpeak> /*goal_handle*/)
{
  logger_.info("Speech cancel requested");
  SoundControl::Abort();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TtsInterface::handleAccepted(const std::shared_ptr<GoalHandleSpeak> goal_handle)
{
  // Execute in a separate thread to avoid blocking the executor
  std::thread{std::bind(&TtsInterface::execute, this, goal_handle)}.detach();
}

void TtsInterface::execute(const std::shared_ptr<GoalHandleSpeak> goal_handle)
{
  auto result = std::make_shared<Speak::Result>();
  auto feedback = std::make_shared<Speak::Feedback>();
  const auto & text = goal_handle->get_goal()->text;

  // Step 1: Synthesize speech (blocking)
  feedback->status = "Synthesizing speech...";
  goal_handle->publish_feedback(feedback);

  int8_t res = TtsControl::produce(text);
  if (res < 0) {
    result->success = false;
    result->message = "TTS synthesis failed with code " + std::to_string(res);
    goal_handle->abort(result);
    logger_.error("TTS produce failed: {}", res);
    return;
  }

  // Check for cancellation
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during synthesis";
    goal_handle->canceled(result);
    return;
  }

  // Step 2: Play the generated audio (non-blocking, wait for completion)
  feedback->status = "Playing speech...";
  goal_handle->publish_feedback(feedback);

  playback_done_ = false;
  playback_success_ = false;

  res = SoundControl::play(tts_output_path_, SOUND_ID);
  if (res < 0) {
    result->success = false;
    result->message = "Sound playback failed with code " + std::to_string(res);
    goal_handle->abort(result);
    logger_.error("SoundControl play failed: {}", res);
    return;
  }

  // Wait for playback to complete
  {
    std::unique_lock<std::mutex> lock(playback_mutex_);
    playback_cv_.wait(lock, [this] { return playback_done_.load(); });
  }

  // Check for cancellation
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during playback";
    goal_handle->canceled(result);
    return;
  }

  if (playback_success_) {
    result->success = true;
    result->message = "Speech completed";
    goal_handle->succeed(result);
    logger_.info("Speech completed for: \"{}\"", text);
  } else {
    result->success = false;
    result->message = "Playback error";
    goal_handle->abort(result);
    logger_.error("Playback error for: \"{}\"", text);
  }
}

void TtsInterface::onSoundComplete(uint16_t id)
{
  logger_.debug("Sound complete: id={}", id);
  {
    std::lock_guard<std::mutex> lock(playback_mutex_);
    playback_done_ = true;
    playback_success_ = true;
  }
  playback_cv_.notify_one();
}

void TtsInterface::onSoundError(uint16_t id)
{
  logger_.error("Sound error: id={}", id);
  {
    std::lock_guard<std::mutex> lock(playback_mutex_);
    playback_done_ = true;
    playback_success_ = false;
  }
  playback_cv_.notify_one();
}

}  // namespace tts_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  tts_interface::TtsInterface tts_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(tts_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  SoundEvent::RemoveListenerOnComplete(&tts_interface::TtsInterface::onSoundCompleteStatic);
  SoundEvent::RemoveListenerOnError(&tts_interface::TtsInterface::onSoundErrorStatic);
  SoundControl::dispose();
  TtsControl::dispose();

  return EXIT_SUCCESS;
}
