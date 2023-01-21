#include <rclcpp/rclcpp.hpp>
#include "turtlebot_teleop/turtlebot_teleop_node.hpp"

using namespace liors_turtle::turtlebot3;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<TurtlebotTeleopNode> turtlebot_teleop_node = std::make_shared<TurtlebotTeleopNode>();

    turtlebot_teleop_node->init();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(turtlebot_teleop_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}