from launch import LaunchDescription
from launch_ros.actions import Node

def  generate_launch_description():
    robot_namespaces_list = ['dobby1', 'dobby2']
    return LaunchDescription([
        Node(
            package="javis_rcs",
            executable="pickup_book",
            name="pickup_book",
            output="screen",
            parameters=[{
                'robot_namespaces': robot_namespaces_list
            }]
        )
    ])