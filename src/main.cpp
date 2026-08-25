#include "ks103_ultrasound/ks103_node.hpp"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ks103_ultrasound::Ks103Node>());
  rclcpp::shutdown();
  return 0;
}
