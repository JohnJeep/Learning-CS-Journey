#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <chrono>
#include <thread>
#include <functional>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("process_file_server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
