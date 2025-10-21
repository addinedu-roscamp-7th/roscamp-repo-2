from launch import LaunchDescription
from launch_ros.actions import Node

def  generate_launch_description():
    return LaunchDescription([
        Node(
            package="javis_rcs",
            executable="pickup_book",
            name="pickup_book",
            output="screen",
        )
    ])