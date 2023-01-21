#ifndef TURTLEBOT_TELEOP_NODE_HPP
#define TURTLEBOT_TELEOP_NODE_HPP

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
            // Members

            // Methods
            void joy_sub_cb_(const sensor_msgs::msg::Joy::SharedPtr msg);

            // Subscribers
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

            // publishers
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

            // Timers
            rclcpp::TimerBase::SharedPtr twist_pub_timer_;

            // Timer callbacks
            void twist_pub_cb();

            /* data */
        };
    }
}

#endif