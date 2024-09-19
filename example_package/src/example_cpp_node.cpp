#include "rclcpp/rclcpp.hpp"

class ExampleCppNode : public rclcpp::Node
{
public:
  ExampleCppNode() : Node("example_cpp_node")
  {
    RCLCPP_INFO(this->get_logger(), "Example C++ node has started!");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleCppNode>());
  rclcpp::shutdown();
  return 0;
}
