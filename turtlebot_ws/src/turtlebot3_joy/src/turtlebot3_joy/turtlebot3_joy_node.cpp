#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "turtlebot3_joy/turtlebot3_joy_node.hpp"

using namespace liors_turtle::turtlebot3;
using namespace std::chrono_literals;

JoyTeleopNode::JoyTeleopNode() : Node("turtlebot_teleop_node")
{
    // declare variables
    this->declare_parameter<int>("joy_linear_x_axis", 1);
    this->declare_parameter<int>("joy_angular_z_axis", 2);
    this->declare_parameter<int>("enable_button", 0);
    this->declare_parameter<float>("max_linear_velocity", 0.22);
    this->declare_parameter<float>("max_angular_velocity", 3.84);
    this->declare_parameter<float>("linear_acceleration", 0.11);
    this->declare_parameter<float>("angular_acceleration", 1.92);
    this->declare_parameter<int>("publish_frequency", 20);

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

bool JoyTeleopNode::init()
{
    // update ros parameters
    publish_frequency_ = this->get_parameter("publish_frequency").as_int();
    enable_button_ = this->get_parameter("enable_button").as_int();
    joy_linear_x_axis_ = this->get_parameter("joy_linear_x_axis").as_int();
    joy_angular_z_axis_ = this->get_parameter("joy_angular_z_axis").as_int();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    linear_acceleration_ = this->get_parameter("linear_acceleration").as_double();
    angular_acceleration_ = this->get_parameter("angular_acceleration").as_double();

    // initiate timers
    twist_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency_)),
                                               std::bind(&JoyTeleopNode::twist_pub_cb, this));

    return true;
}

bool JoyTeleopNode::convert_joy_to_vin(sensor_msgs::msg::Joy joy_msg)
{
    target_velocity_.linear.x = joy_msg.axes[joy_linear_x_axis_] * max_linear_velocity_;
    target_velocity_.angular.z = joy_msg.axes[joy_angular_z_axis_] * max_angular_velocity_;
    return true;
}

void JoyTeleopNode::joy_sub_cb_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    target_velocity_.linear.x = (double)(msg->axes[joy_angular_z_axis_] * max_angular_velocity_);
    target_velocity_.angular.z = (double)(msg->axes[joy_linear_x_axis_] * max_linear_velocity_);

    set_twist_publisher_state(msg->buttons[enable_button_]);
}

void JoyTeleopNode::twist_pub_cb()
{
    update_current_velocity();

    twist_pub_->publish(current_velocity_);
}

void JoyTeleopNode::update_current_velocity()
{
    std::vector<double> velocity_diff(2, 0.0);
    std::vector<double> delta_v(2, 0.0);
    double dt = 1 / static_cast<double>(publish_frequency_);

    velocity_diff[0] = target_velocity_.linear.x - current_velocity_.linear.x;
    velocity_diff[1] = target_velocity_.angular.z - current_velocity_.angular.z;

    delta_v[0] = linear_acceleration_ * dt;
    delta_v[1] = angular_acceleration_ * dt;

    for (int i = 0; i < 2; i++)
    {
        if (std::signbit(velocity_diff[i]))
        {
            delta_v[i] *= -1;
        }
    }

    current_velocity_.linear.x += delta_v[0];
    current_velocity_.angular.z += delta_v[1];
}

inline void JoyTeleopNode::set_twist_publisher_state(int enable_button_state)
{
    if (enable_button_state == 0 && armed_ == true)
    {
        armed_ = false;

        target_velocity_.linear.x = 0.0;
        target_velocity_.angular.z = 0.0;
        current_velocity_ = target_velocity_;

        twist_pub_->publish(current_velocity_);
        twist_pub_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Twist publisher disabled!");
    }
    else if (enable_button_state == 1 && armed_ == false)
    {
        armed_ = true;
        twist_pub_timer_->reset();
        RCLCPP_INFO(this->get_logger(), "Twist publisher enabled!");
    }
    else
    {
        // do nothing...
    }
}