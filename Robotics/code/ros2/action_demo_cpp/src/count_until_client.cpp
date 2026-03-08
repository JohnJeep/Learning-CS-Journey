// formate: <package_name/msg/msg_type.hpp>
#include "custom_interfaces/action/count_until.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <memory>

using CountUntil = custom_interfaces::action::CountUntil;
using GoalHandleCountUntil = rclcpp_action::ClientGoalHandle<CountUntil>;

std::atomic<bool> g_sigint_requested{false};

extern "C" void sigint_handler(int)
{
  // Signal handler里只做最小操作
  g_sigint_requested.store(true);
}

class CountUntilActionClient : public rclcpp::Node
{
 public:
  CountUntilActionClient()
    : Node("count_until_client")
  {
    RCLCPP_INFO(this->get_logger(), "constructor");
    this->action_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");

    // 周期检查是否按下 Ctrl+C，触发 cancel
    this->sigint_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CountUntilActionClient::on_sigint_check, this));
  }

  bool send_goal(int target)
  {
    using namespace std::placeholders;
    if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_INFO(this->get_logger(), "wait for action server timeout 10s");
      return false;
    }

    auto goal_msg = CountUntil::Goal();
    goal_msg.target = target;
    RCLCPP_INFO(this->get_logger(), "send goal with target %d", target);

    auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
    options.goal_response_callback = std::bind(&CountUntilActionClient::goal_response_callback, this, _1);
    options.feedback_callback = std::bind(&CountUntilActionClient::feedback_callback, this, _1, _2);
    options.result_callback = std::bind(&CountUntilActionClient::result_callback, this, _1);

    this->action_client_->async_send_goal(goal_msg, options);
    return true;
  }

 private:
  void on_sigint_check()
  {
    if (!g_sigint_requested.load()) {
      return;
    }

    // 只处理一次
    if (this->sigint_handled_.exchange(true)) {
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Ctrl+C detected, requesting cancel...");

    // 若 goal 还没被接受，无法 cancel，直接退出
    if (!this->goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "goal handle not ready, shutdown directly");
      rclcpp::shutdown();
      return;
    }

    if (this->cancel_sent_.exchange(true)) {
      return;
    }

    // 发送 cancel，等待 result_callback 再 shutdown
    auto future_cancel = this->action_client_->async_cancel_goal(this->goal_handle_);
    (void)future_cancel;
  }

  void goal_response_callback(const GoalHandleCountUntil::SharedPtr &goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
      rclcpp::shutdown();
    } else {
      this->goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleCountUntil::SharedPtr,
    const std::shared_ptr<const CountUntil::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "received feedback: %d", feedback->current_number);
  }

  void result_callback(const GoalHandleCountUntil::WrappedResult &result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "result: final_number = %d", result.result->final_number);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "goal was canceled, final_number = %d", result.result->final_number);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "unknown result code");
        break;
    }

    // 统一优雅退出
    rclcpp::shutdown();
  }

 private:
  rclcpp_action::Client<CountUntil>::SharedPtr action_client_;
  GoalHandleCountUntil::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr sigint_timer_;
  std::atomic<bool> cancel_sent_{false};
  std::atomic<bool> sigint_handled_{false};
};

int main(int argc, char **argv)
{
  // 1) 关闭 rclcpp 默认 SIGINT 处理，避免按 Ctrl+C 直接 shutdown
  rclcpp::InitOptions init_options;
  rclcpp::init(
    argc, argv, init_options,
    rclcpp::SignalHandlerOptions::None);

  // 2) 安装我们自己的 SIGINT handler
  std::signal(SIGINT, sigint_handler);

  auto client = std::make_shared<CountUntilActionClient>();

  // get target from command line, default to 5
  int target = 5;
  if (argc > 1) {
    target = std::stoi(argv[1]);
  }

  if (!client->send_goal(target)) {
    rclcpp::shutdown();
    RCLCPP_ERROR(client->get_logger(), "client exited");
    return 1;
  }

  RCLCPP_INFO(client->get_logger(), "goal sent, press Ctrl+C to cancel gracefully");
  rclcpp::spin(client);

  // spin 返回后保证 context 已关闭
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
