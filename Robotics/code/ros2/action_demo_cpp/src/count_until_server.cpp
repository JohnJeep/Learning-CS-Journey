#include "action_demo_cpp/action/count_until.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <chrono>
#include <thread>
#include <functional>

using CountUntil = action_demo_cpp::action::CountUntil;

class CountUntilActionServer : public rclcpp::Node
{
 public:
  using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;
  CountUntilActionServer()
    : Node("count_until_action_server")
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
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const CountUntil::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "received goal request with target %d", goal->target);
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "received cancel goal");
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

    rclcpp::Rate rate(10);
    for (size_t i = 1; i <= goal->target; i++) {
      if (goal_handle->is_canceling()) {
        result->final_number = i - 1;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "goal cancelled at %d", i - 1);
        return;
      }
      
      feedback->current_number = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "publish feedback: %d", i);

      rate.sleep();
    }

    result->final_number = goal->target;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "goal success");
  }
  
 private:
  rclcpp_action::Server<CountUntil>::SharedPtr action_server_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountUntilActionServer>("count_until_server");
  rclcpp::spin((node));
  rclcpp::shutdown();

  return 0;
}