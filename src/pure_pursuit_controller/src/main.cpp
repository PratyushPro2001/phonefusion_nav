#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pure_pursuit");
  RCLCPP_INFO(node->get_logger(), "pure_pursuit stub running.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
