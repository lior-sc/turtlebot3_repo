#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "turtlebot3_joy/turtlebot3_joy_node.hpp"

using namespace liors_turtle::turtlebot3;
using namespace std::chrono_literals;

JoyTeleopNode::JoyTeleopNode() : Node("turtlebot_teleop_node")
{
    // declare variables
    declare_node_parameters();

    // create publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // create subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "~/joy", 1, std::bind(&JoyTeleopNode::joy_sub_cb_, this, std::placeholders::_1));
}

JoyTeleopNode::~JoyTeleopNode()
{
    RCLCPP_INFO(this->get_logger(), "turtlebot_teleop_node is closing. zero velocity command will be sent!");
    geometry_msgs::msg::Vector3 zero_vel;
    zero_vel.x = 0;
    zero_vel.y = 0;
    zero_vel.z = 0;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.set__linear(zero_vel);
    cmd_vel.set__angular(zero_vel);

    // stop the robot when distroying th node
    twist_pub_->publish(cmd_vel);
}

inline void JoyTeleopNode::declare_node_parameters()
{
    this->declare_parameter<int>("joystick_lin_x_axis", 2);
    this->declare_parameter<int>("joystick_ang_z_axis", 2);
    this->declare_parameter<float>("turtlebot_max_lin_vel", 0.22);
    this->declare_parameter<float>("turtlebot_max_ang_vel", 3.84);
}

inline void JoyTeleopNode::update_parameters_from_config()
{
    this->get_parameter("joystick_lin_x_axis", joystick_lin_x_axis_);
    this->get_parameter("joystick_ang_z_axis", joystick_ang_z_axis_);
}

bool JoyTeleopNode::init()
{
    // initialize timers
    // twist_pub_timer_ = this->create_wall_timer(20ms, std::bind(&JoyTeleopNode::twist_pub_cb, this));
    update_parameters_from_config();
    return true;
}

bool JoyTeleopNode::convert_joy_to_vin(sensor_msgs::msg::Joy joy_msg, geometry_msgs::msg::Twist &twist_value)
{
    twist_value.linear.x = joy_msg.axes[joystick_lin_x_axis_] * turtlebot_max_lin_vel;
    twist_value.angular.z = joy_msg.axes[joystick_ang_z_axis_] * turtlebot_max_ang_vel;
    return true;
}

void JoyTeleopNode::joy_sub_cb_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = (double)(msg->axes[joystick_ang_z_axis_] * turtlebot_max_ang_vel);
    cmd_vel.angular.z = (double)(msg->axes[joystick_lin_x_axis_] * turtlebot_max_lin_vel);

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "linear_speed: "
                                    << cmd_vel.linear.x << "  "
                                    << "angular speed: " << cmd_vel.linear.x);

    twist_pub_->publish(std::move(cmd_vel));
}
