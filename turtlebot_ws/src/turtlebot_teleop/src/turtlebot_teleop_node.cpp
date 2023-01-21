#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "turtlebot_teleop/turtlebot_teleop_node.hpp"

using namespace liors_turtle::turtlebot3;
using namespace std::chrono_literals;

TurtlebotTeleopNode::TurtlebotTeleopNode() : Node("turtlebot_teleop_node")
{
    // declare variables

    // create publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // create subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "~/joy", 1, std::bind(&TurtlebotTeleopNode::joy_sub_cb_, this, std::placeholders::_1));
}

bool TurtlebotTeleopNode::init()
{
    // initialize timers
    twist_pub_timer_ = this->create_wall_timer(20 * 1ms,
                                               std::bind(&TurtlebotTeleopNode::twist_pub_cb, this));
}
