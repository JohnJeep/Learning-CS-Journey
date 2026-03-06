#include "action_demo_cpp/action/count_until.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <chrono>
#include <thread>
#include <functional>

using CountUntil = action_demo_cpp::action::CountUntil;
using GoalHandleCountUntil = rclcpp_action::ClientGoalHandle<CountUntil>;

class CountUntilActionClient : public rclcpp::Node
{
 public:
  CountUntilActionClient()
    : Node("count_until_action_client")
  {
    RCLCPP_INFO(this->get_logger(), "constructor");
    this->action_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
  }

  void send_goal(int target)
  {
    using namespace std::placeholders;
    if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_INFO(this->get_logger(), "wait for action timeout");
      return;
    }

    auto goal_msg = CountUntil::Goal();
    goal_msg.target = target;
    RCLCPP_INFO(this->get_logger(), "send goal with target %d", target);

    auto send_goal_options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&CountUntilActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&CountUntilActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&CountUntilActionClient::result_callback, this, _1);

    this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  void goal_response_callback(const GoalHandleCountUntil::SharedPtr &goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleCountUntil::SharedPtr goal, const std::shared_ptr<const CountUntil::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "received feedback: %d", feedback->current_number);
  }

  void result_callback(const GoalHandleCountUntil::WrappedResult &result)
  {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "result: final_number = %d", result.result->final_number);
    rclcpp::shutdown();
  }

 private:
  rclcpp_action::Client<CountUntil>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<CountUntilActionClient>("count_until_action_client");

  int target = 5;
  if (argc > 1) {
    target = std::stoi(argv[1]);
  }
  client->send_goal(target);

  rclcpp::spin((client));
  rclcpp::shutdown();
  
  return 0;
}
