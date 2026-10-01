from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hikrobot_camera',
            executable='hik_camera_node',
            name='hikrobot_camera_node',
            output='screen',
            parameters=[
                {'exposure_time': 5000.0},
                {'gain': 10.0},
                {'frame_rate': 30.0}
            ]
        )
    ])