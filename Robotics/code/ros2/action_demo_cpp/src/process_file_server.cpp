#include "custom_interfaces/action/process_file.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <functional>
#include <memory>
#include <thread>
#include <fstream>

using ProcessFile = custom_interfaces::action::ProcessFile;
using GoalHandleProcessFile = rclcpp_action::ServerGoalHandle<ProcessFile>;

class ProcessFileActionServer : public rclcpp::Node
{
public:
  ProcessFileActionServer() : Node("process_file_server")
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ProcessFile>(
      this,
      "process_file",
      std::bind(&ProcessFileActionServer::handle_goal, this, _1, _2),
      std::bind(&ProcessFileActionServer::handle_cancel, this, _1),
      std::bind(&ProcessFileActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ProcessFile>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ProcessFile::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "received goal request with file_path: %s", goal->file_path.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleProcessFile> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleProcessFile> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&ProcessFileActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleProcessFile> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "executing file processing...");
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ProcessFile::Result>();

    // 模拟文件处理：检查文件是否存在（这里简单检查文件是否存在）
    std::ifstream f(goal->file_path.c_str());
    if (f.good()) {
      result->success = true;
      result->message = "file exists and processed (simulated).";
      RCLCPP_INFO(this->get_logger(), "file processing succeeded");
    } else {
      result->success = false;
      result->message = "file not found.";
      RCLCPP_ERROR(this->get_logger(), "file not found: %s", goal->file_path.c_str());
    }

    // 直接设置结果，没有反馈
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessFileActionServer>());
  rclcpp::shutdown();

  return 0;
}
