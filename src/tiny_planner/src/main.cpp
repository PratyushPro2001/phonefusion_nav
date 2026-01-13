#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tiny_planner");
  RCLCPP_INFO(node->get_logger(), "tiny_planner stub running.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
