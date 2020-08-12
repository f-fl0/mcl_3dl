#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_ptr = std::make_shared<rclcpp::Node>("dummy");
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}