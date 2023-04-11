#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "turtlebot3_joy/turtlebot3_joy_node.hpp"

using namespace liors_turtle::turtlebot3;
using namespace std::chrono_literals;

JoyTeleopNode::JoyTeleopNode() : Node("turtlebot_teleop_node"),
                                 armed_(false)
{
    // declare variables
    this->declare_parameter<int>("joy_linear_x_axis", 1);
    this->declare_parameter<int>("joy_angular_z_axis", 0);
    this->declare_parameter<int>("enable_button", 0);
    this->declare_parameter<float>("max_linear_velocity", 0.22);
    this->declare_parameter<float>("max_angular_velocity", 3.84);
    this->declare_parameter<float>("linear_acceleration", 0.44);
    this->declare_parameter<float>("angular_acceleration", 7.68);
    this->declare_parameter<int>("publish_frequency", 20);

    // create publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // create subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&JoyTeleopNode::joy_sub_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "System disarmed");
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
    RCLCPP_INFO(this->get_logger(), "retrieve ros parameter values");
    publish_frequency_ = this->get_parameter("publish_frequency").as_int();
    enable_button_ = this->get_parameter("enable_button").as_int();
    joy_linear_x_axis_ = this->get_parameter("joy_linear_x_axis").as_int();
    joy_angular_z_axis_ = this->get_parameter("joy_angular_z_axis").as_int();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    linear_acceleration_ = this->get_parameter("linear_acceleration").as_double();
    angular_acceleration_ = this->get_parameter("angular_acceleration").as_double();

    // initiate timers
    RCLCPP_INFO(this->get_logger(), "create wall timer");
    twist_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency_)),
                                               std::bind(&JoyTeleopNode::twist_pub_cb, this));
    twist_pub_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "cancel wall timer");

    return true;
}

bool JoyTeleopNode::convert_joy_to_vin(sensor_msgs::msg::Joy joy_msg)
{
    target_velocity_.linear.x = joy_msg.axes[joy_linear_x_axis_] * max_linear_velocity_;
    target_velocity_.angular.z = joy_msg.axes[joy_angular_z_axis_] * max_angular_velocity_;
    return true;
}

void JoyTeleopNode::joy_sub_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "hello from subscriber");
    target_velocity_.angular.z = static_cast<double>(msg->axes[joy_angular_z_axis_] * max_angular_velocity_);
    target_velocity_.linear.x = static_cast<double>(msg->axes[joy_linear_x_axis_] * max_linear_velocity_);

    set_twist_publisher_state(msg->buttons[enable_button_]);
}

void JoyTeleopNode::twist_pub_cb()
{
    update_current_velocity();

    twist_pub_->publish(current_velocity_);
    RCLCPP_INFO(this->get_logger(), "publishing x: %.3f, w: %.3f", current_velocity_.linear.x, current_velocity_.angular.z);
}

void JoyTeleopNode::update_current_velocity()
{
    std::vector<double> velocity_diff(2, 0.0);
    std::vector<double> delta_v(2, 0.0);
    std::vector<double> acc{linear_acceleration_, angular_acceleration_};

    double dt = 1 / static_cast<double>(publish_frequency_);

    velocity_diff[0] = target_velocity_.linear.x - current_velocity_.linear.x;
    velocity_diff[1] = target_velocity_.angular.z - current_velocity_.angular.z;

    for (int i = 0; i < 2; i++)
    {
        if (abs(velocity_diff[i]) > delta_v[i])
        {
            if (std::signbit(velocity_diff[i]))
            {

                // this means that the velocity difference is negative
                delta_v[i] = -acc[i] * dt;
            }
            else
            {
                delta_v[i] = acc[i] * dt;
            }
        }
        else
        {
            delta_v[i] = velocity_diff[i];
        }
    }

    current_velocity_.linear.x += delta_v[0];
    current_velocity_.angular.z += delta_v[1];
}

void JoyTeleopNode::set_twist_publisher_state(int enable_button_state)
{
    // RCLCPP_INFO(this->get_logger(), "set twist publisher state, enable_button_state = %d, armed_ = %d", enable_button_state, armed_);
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