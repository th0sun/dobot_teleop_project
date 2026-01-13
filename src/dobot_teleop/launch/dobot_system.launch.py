from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. เปิด Node ขับหุ่น
        Node(
            package='dobot_teleop',
            executable='dobot_driver.py',
            name='dobot_driver',
            output='screen'
        ),
        # 2. เปิด Node รับ TCP (เปิดต่อกันเลย)
        Node(
            package='dobot_teleop',
            executable='tcp_receiver.py',
            name='tcp_receiver',
            output='screen'
        )
    ])
