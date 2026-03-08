#include "custom_interfaces/action/process_file.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <functional>
#include <memory>
#include <thread>


using ProcessFile = custom_interfaces::action::ProcessFile;
using GoalHandleProcessFile = rclcpp_action::ClientGoalHandle<ProcessFile>;

class ProcessFileActionClient : public rclcpp::Node
{
public:
  ProcessFileActionClient() : Node("process_file_client")
  {
    this->client_ptr_ = rclcpp_action::create_client<ProcessFile>(this, "process_file");
  }

  void send_goal(const std::string & file_path)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "wating for action server 10s timeout");
      return;
    }

    auto goal_msg = ProcessFile::Goal();
    goal_msg.file_path = file_path;

    RCLCPP_INFO(this->get_logger(), "sending goal for file: %s", file_path.c_str());

    auto options = rclcpp_action::Client<ProcessFile>::SendGoalOptions();
    options.goal_response_callback =
      std::bind(&ProcessFileActionClient::goal_response_callback, this, _1);
    // 没有反馈回调，因为该 Action 没有 feedback 字段
    options.result_callback =
      std::bind(&ProcessFileActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<ProcessFile>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandleProcessFile::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
    }
  }

  void result_callback(const GoalHandleProcessFile::WrappedResult & result)
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
    RCLCPP_INFO(this->get_logger(), "result: success = %d, message = '%s'",
                result.result->success, result.result->message.c_str());
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<ProcessFileActionClient>();

  std::string file_path = "/home/jacky/test.txt"; // 默认测试文件
  if (argc > 1) {
    file_path = argv[1];
  }
  client->send_goal(file_path);

  rclcpp::spin(client);
  rclcpp::shutdown();
  return 0;
}
