#include "doly_hw_interfaces/eye_interface.hpp"

#include <functional>
#include <string>
#include <thread>

namespace eye_interface
{
EyeInterface * EyeInterface::instance_ = nullptr;

EyeInterface::EyeInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("eye_interface", options), logger_(this->get_logger())
{
  instance_ = this;

  // *** IMPORTANT ***
  // Stop doly service if running,
  // otherwise instance of libraries cause conflict
  if (Helper::stopDolyService() < 0) {
    logger_.error("Doly service stop failed");
    return;
  }

  logger_.info("EyeControl Version:{:.3f}", EyeControl::getVersion());

  if (EyeControl::init(ColorCode::BLUE, ColorCode::WHITE) != 0) {
    logger_.error("EyeControl init failed");
    return;
  }

  // Start with default visuals on the eyes to indicate ROS stack has started
  VContent visualL = VContent::getImage("/assets/images/ROS2.png", true, true);
  VContent visualR = VContent::getImage("/assets/images/doly_glow.png", true, true);
  if (!visualL.isReady() || !visualR.isReady())
    logger_.error("image load failed!");
  else
  {
    int retL = EyeControl::setIris(&visualL, EyeSide::LEFT); // load content for left eyelid
    int retR = EyeControl::setIris(&visualR, EyeSide::RIGHT); // load content for right eyelid
    if (retL < 0)
      logger_.error("Set left eye lid failed err:{}", retL);
    if (retR < 0)
      logger_.error("Set right eye lid failed err:{}", retR);
  }

  EyeEvent::AddListenerOnStart(&EyeInterface::onEyeStartStatic);
  EyeEvent::AddListenerOnComplete(&EyeInterface::onEyeCompleteStatic);
  EyeEvent::AddListenerOnAbort(&EyeInterface::onEyeAbortStatic);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<EyeAnimation>(
    this, "set_eye_animation",
    std::bind(&EyeInterface::handleGoal, this, _1, _2),
    std::bind(&EyeInterface::handleCancel, this, _1),
    std::bind(&EyeInterface::handleAccepted, this, _1));

  set_eye_type_service_ = this->create_service<doly_msgs::srv::SetEyeType>(
    "set_eye_type", [this](
                      const std::shared_ptr<doly_msgs::srv::SetEyeType::Request> request,
                      std::shared_ptr<doly_msgs::srv::SetEyeType::Response> response) {
      this->setEyeTypeCallback(request, response);
    });

  logger_.info("EyeInterface has been started.");
}

rclcpp_action::GoalResponse EyeInterface::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const EyeAnimation::Goal> goal)
{
  if (goal->animation.empty()) {
    logger_.warn("Rejecting empty eye animation goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (EyeControl::isAnimating()) {
    logger_.warn("Rejecting eye animation goal while another animation is running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  logger_.info("Received eye animation goal: {}", goal->animation);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EyeInterface::handleCancel(
  const std::shared_ptr<GoalHandleEyeAnimation> /*goal_handle*/)
{
  logger_.info("Eye animation cancel requested");
  EyeControl::Abort();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EyeInterface::handleAccepted(const std::shared_ptr<GoalHandleEyeAnimation> goal_handle)
{
  // Execute in a separate thread to avoid blocking the executor
  std::thread{std::bind(&EyeInterface::execute, this, goal_handle)}.detach();
}

void EyeInterface::execute(const std::shared_ptr<GoalHandleEyeAnimation> goal_handle)
{
  auto result = std::make_shared<EyeAnimation::Result>();
  auto feedback = std::make_shared<EyeAnimation::Feedback>();
  const auto & animation = goal_handle->get_goal()->animation;

  feedback->status = "Starting eye animation...";
  goal_handle->publish_feedback(feedback);

  const uint16_t id = ++active_animation_id_;
  animation_done_ = false;
  animation_success_ = false;

  int8_t res = EyeControl::setAnimation(id, animation);
  if (res < 0) {
    result->success = false;
    result->message = "Unknown or invalid animation name: " + animation;
    goal_handle->abort(result);
    logger_.error("EyeControl setAnimation failed for '{}', code={}", animation, res);
    return;
  }

  {
    std::unique_lock<std::mutex> lock(animation_mutex_);
    animation_cv_.wait(lock, [this] { return animation_done_.load(); });
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled";
    goal_handle->canceled(result);
    return;
  }

  if (animation_success_) {
    result->success = true;
    result->message = "Animation completed";
    goal_handle->succeed(result);
    logger_.info("Eye animation completed: {}", animation);
  } else {
    result->success = false;
    result->message = "Animation aborted";
    goal_handle->abort(result);
    logger_.warn("Eye animation aborted: {}", animation);
  }
}

void EyeInterface::setEyeTypeCallback(
  const std::shared_ptr<doly_msgs::srv::SetEyeType::Request> request,
  std::shared_ptr<doly_msgs::srv::SetEyeType::Response> response)
{
  if (!isValidIrisShape(request->iris_shape)) {
    response->success = false;
    response->message = "Invalid iris_shape value: " + std::to_string(request->iris_shape);
    logger_.warn("Rejected set_eye_type request with invalid iris_shape={}", request->iris_shape);
    return;
  }

  if (!isValidColorCode(request->iris_color)) {
    response->success = false;
    response->message = "Invalid iris_color value: " + std::to_string(request->iris_color);
    logger_.warn("Rejected set_eye_type request with invalid iris_color={}", request->iris_color);
    return;
  }

  if (!isValidColorCode(request->background_color)) {
    response->success = false;
    response->message =
      "Invalid background_color value: " + std::to_string(request->background_color);
    logger_.warn(
      "Rejected set_eye_type request with invalid background_color={}",
      request->background_color);
    return;
  }

  const auto iris_shape = static_cast<IrisShape>(request->iris_shape);
  const auto iris_color = static_cast<ColorCode>(request->iris_color);
  const auto background_color = static_cast<ColorCode>(request->background_color);

  const int8_t result = EyeControl::setEyes(iris_shape, iris_color, background_color);
  if (result < 0) {
    response->success = false;
    response->message = "EyeControl::setEyes failed with code " + std::to_string(result);
    logger_.error(
      "set_eye_type failed: shape={}, iris_color={}, background_color={}, code={}",
      request->iris_shape, request->iris_color, request->background_color, result);
    return;
  }

  response->success = true;
  response->message = "Eye type updated";
  logger_.info(
    "Eye type updated: shape={}, iris_color={}, background_color={}",
    request->iris_shape, request->iris_color, request->background_color);
}

void EyeInterface::onEyeStart(uint16_t id)
{
  logger_.debug("Eye animation started id={}", id);
}

void EyeInterface::onEyeComplete(uint16_t id)
{
  if (id != active_animation_id_) {
    return;
  }
  logger_.debug("Eye animation complete id={}", id);
  {
    std::lock_guard<std::mutex> lock(animation_mutex_);
    animation_done_ = true;
    animation_success_ = true;
  }
  animation_cv_.notify_one();
}

void EyeInterface::onEyeAbort(uint16_t id)
{
  if (id != active_animation_id_) {
    return;
  }
  logger_.debug("Eye animation abort id={}", id);
  {
    std::lock_guard<std::mutex> lock(animation_mutex_);
    animation_done_ = true;
    animation_success_ = false;
  }
  animation_cv_.notify_one();
}

}  // namespace eye_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  eye_interface::EyeInterface eye_interface_node;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(eye_interface_node.get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  // Cleanup
  EyeEvent::RemoveListenerOnStart(&eye_interface::EyeInterface::onEyeStartStatic);
  EyeEvent::RemoveListenerOnComplete(&eye_interface::EyeInterface::onEyeCompleteStatic);
  EyeEvent::RemoveListenerOnAbort(&eye_interface::EyeInterface::onEyeAbortStatic);
  EyeControl::Abort();

  return EXIT_SUCCESS;
}
