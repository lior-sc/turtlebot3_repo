import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

turtlebot3_joy_config = os.path.join(
    get_package_share_directory('turtlebot3_joy'),
    'config',
    'turtlebot3_joy.yaml'
)


def generate_launch_description():
    ld = LaunchDescription()

    joy_publisher_node = Node(
        package="joy",
        executable="joy_node"
    )

    custom_joy_teleop_node = Node(
        package="turtlebot3_joy",
        executable="turtlebot3_joy_node",
        output="screen",
        parameters=[turtlebot3_joy_config]
    )

    ld.add_action(joy_publisher_node)
    ld.add_action(custom_joy_teleop_node)

    return ld
