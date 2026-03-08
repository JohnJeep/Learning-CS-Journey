#include "custom_interfaces/action/count_until.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <chrono>
#include <thread>
#include <functional>

using CountUntil = custom_interfaces::action::CountUntil;

class CountUntilActionServer : public rclcpp::Node
{
 public:
  using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;
  CountUntilActionServer()
    : Node("count_until_server")
  {
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<CountUntil>(
      this,
      "count_until",
      std::bind(&CountUntilActionServer::handle_goal, this, _1, _2),
      std::bind(&CountUntilActionServer::handle_cancel, this, _1),
      std::bind(&CountUntilActionServer::handle_accepted, this, _1));
  }

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "received goal request with target %d", goal->target);
    (void)uuid; 
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "received cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "handle accepted");
    using namespace std::placeholders;
    std::thread{
      std::bind(&CountUntilActionServer::execute, this, _1), goal_handle
    }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "execute goal");

    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CountUntil::Feedback>();
    auto result = std::make_shared<CountUntil::Result>();

    using namespace std::chrono_literals;
    int current = 0;
    while (rclcpp::ok() && current < goal->target) {
      // 先处理取消，尽快退出
      if (goal_handle->is_canceling()) {
        result->final_number = current;
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "goal canceled at: %d", current);
        return;
      }

      // 非 active 状态保护
      if (!goal_handle->is_active()) {
        RCLCPP_WARN(this->get_logger(), "goal is not active, stop execute");
        return;
      }

      ++current;
      feedback->current_number = current;

      // pushlish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "publish feedback: %d", current);

      // 小步进 sleep，提高 cancel 响应速度
      for (int i = 0; i < 10; ++i) {
        if (goal_handle->is_canceling()) {
          result->final_number = current;
          goal_handle->canceled(result);
          RCLCPP_WARN(this->get_logger(), "goal canceled at: %d", current);
          return;
        }
        rclcpp::sleep_for(100ms);
      }
    }
    result->final_number = goal->target;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "goal handle success");
  }
  
 private:
  rclcpp_action::Server<CountUntil>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountUntilActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}