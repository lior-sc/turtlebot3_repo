#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "turtlebot_teleop/turtlebot_teleop_node.hpp"

using namespace liors_turtle::turtlebot3;
using namespace std::chrono_literals;

TurtlebotTeleopNode::TurtlebotTeleopNode() : Node("turtlebot_teleop_node")
{
    // declare variables
    declare_node_parameters();

    // create publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // create subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "~/joy", 1, std::bind(&TurtlebotTeleopNode::joy_sub_cb_, this, std::placeholders::_1));
}

inline void TurtlebotTeleopNode::declare_node_parameters()
{
    this->declare_parameter<int>("joystick_lin_x_axis", 2);
    this->declare_parameter<int>("joystick_ang_z_axis", 2);
    this->declare_parameter<float>("turtlebot_max_lin_vel", 0.22);
    this->declare_parameter<float>("turtlebot_max_ang_vel", 3.84);
}

inline void TurtlebotTeleopNode::update_parameters_from_config()
{
    this->get_parameter("joystick_lin_x_axis", joystick_lin_x_axis_);
    this->get_parameter("joystick_ang_z_axis", joystick_ang_z_axis_);
}

bool TurtlebotTeleopNode::init()
{
    // initialize timers
    twist_pub_timer_ = this->create_wall_timer(20ms,
                                               std::bind(&TurtlebotTeleopNode::twist_pub_cb, this));
    update_parameters_from_config();
}

bool TurtlebotTeleopNode::convert_joy_to_vin(sensor_msgs::msg::Joy joy_msg, geometry_msgs::msg::Twist twist_value)
{
    twist_value.linear.x = joy_msg.axes[joystick_lin_x_axis_] * turtlebot_max_lin_vel;
    twist_value.angular.z = joy_msg.axes[joystick_ang_z_axis_] * turtlebot_max_ang_vel;
}

void TurtlebotTeleopNode::joy_sub_cb_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
}

void TurtlebotTeleopNode::twist_pub_cb()
{
}