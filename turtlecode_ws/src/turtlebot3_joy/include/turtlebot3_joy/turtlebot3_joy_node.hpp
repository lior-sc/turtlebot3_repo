#ifndef TURTLEBOT3_JOY_NODE_HPP
#define TURTLEBOT3_JOY_NODE_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace liors_turtle
{
    namespace turtlebot3
    {
        class JoyTeleopNode : public rclcpp::Node
        {
        public:
            JoyTeleopNode(/* args */);
            ~JoyTeleopNode();

            bool init();

        private:
            // variables
            bool always_armed_;
            int arm_button_;
            int publish_frequency_;
            int joy_linear_x_axis_;
            int joy_angular_z_axis_;
            double max_linear_velocity_;
            double max_angular_velocity_;
            double linear_acceleration_;
            double angular_acceleration_;

            geometry_msgs::msg::Twist current_velocity_;
            geometry_msgs::msg::Twist target_velocity_;
            bool armed_;

            // Members

            // Methods
            bool convert_joy_to_vin(sensor_msgs::msg::Joy joy_msg);
            void update_current_velocity();
            void set_twist_publisher_state(int);

            // Subscribers
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

            // publishers
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

            // Timers
            rclcpp::TimerBase::SharedPtr twist_pub_timer_;

            // Timer callbacks
            void twist_pub_cb();

            // Subscriber callbacks
            void joy_sub_cb(const sensor_msgs::msg::Joy::SharedPtr msg);

            /* data */
        };
    }
}

#endif