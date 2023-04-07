#include <rclcpp/rclcpp.hpp>
#include "turtlebot3_joy/turtlebot3_joy_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto turtlebot_joy_node = std::make_shared<liors_turtle::turtlebot3::JoyNode>();

    turtlebot_joy_node->init();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(turtlebot_joy_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}