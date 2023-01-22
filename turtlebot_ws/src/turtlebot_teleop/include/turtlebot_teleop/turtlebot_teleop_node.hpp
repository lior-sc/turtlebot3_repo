#ifndef TURTLEBOT_TELEOP_NODE_HPP
#define TURTLEBOT_TELEOP_NODE_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace liors_turtle
{
    namespace turtlebot3
    {
        class TurtlebotTeleopNode : public rclcpp::Node
        {
        public:
            TurtlebotTeleopNode(/* args */);
            ~TurtlebotTeleopNode();

            bool init();

        private:
            // variables
            int joystick_lin_x_axis_;
            int joystick_ang_z_axis_;
            float turtlebot_max_lin_vel;
            float turtlebot_max_ang_vel;

            // Members

            // Methods
            inline void declare_node_parameters();
            inline void update_parameters_from_config();
            bool convert_joy_to_vin(sensor_msgs::msg::Joy joy_msg, geometry_msgs::msg::Twist twist_value);

            // Subscribers
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

            // publishers
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

            // Timers
            rclcpp::TimerBase::SharedPtr twist_pub_timer_;

            // Timer callbacks
            void twist_pub_cb();

            // Subscriber callbacks
            void joy_sub_cb_(const sensor_msgs::msg::Joy::SharedPtr msg);

            /* data */
        };
    }
}

#endif