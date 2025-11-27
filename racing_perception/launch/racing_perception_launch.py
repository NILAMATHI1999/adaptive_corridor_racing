from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Fake LiDAR publisher
        Node(
            package='racing_perception',
            executable='fake_lidar_node',
            name='fake_lidar_node',
            output='screen'
        ),

        # Perception + steering + behavior
        Node(
            package='racing_perception',
            executable='racing_perception_node',
            name='racing_perception_node',
            output='screen'
        ),

        # Speed + steering controller
        Node(
            package='racing_perception',
            executable='speed_controller_node',
            name='speed_controller_node',
            output='screen'
        ),

        # NEW: Logger node
        Node(
            package='racing_perception',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen'
        ),
    ])
