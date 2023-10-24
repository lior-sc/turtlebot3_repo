from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    joy_publisher_node = Node(
        package="joy",
        executable="joy_node"
    )

    joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        output="screen",
        parameters=[
            {"enable_button": 7},
            {"axis_linear.x": 1},
            {"axis_angular.yaw": 0},
            {"scale_linear.x": 0.22},
            {"scale_angular.yaw": 3.84},
        ]
    )

    ld.add_action(joy_publisher_node)
    ld.add_action(joy_teleop_node)

    return ld
