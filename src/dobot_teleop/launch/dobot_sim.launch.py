import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. หาไฟล์โมเดล (Xacro) ---
    description_pkg = get_package_share_directory('mg400_description')
    xacro_file = os.path.join(description_pkg, 'urdf', 'mg400.urdf.xacro')

    # --- 2. แปลง Xacro เป็น URDF ---
    robot_description_content = Command(['xacro ', xacro_file])

    # --- 3. Robot State Publisher (คนคำนวณข้อต่อ) ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # --- 4. RViz2 (คนแสดงผล) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # --- 5. ระบบของเรา (Driver + Receiver) ---
    driver_node = Node(
        package='dobot_teleop',
        executable='dobot_driver.py',
        name='dobot_driver',
        output='screen'
    )
    
    receiver_node = Node(
        package='dobot_teleop',
        executable='tcp_receiver.py',
        name='tcp_receiver',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        driver_node,
        receiver_node
    ])
