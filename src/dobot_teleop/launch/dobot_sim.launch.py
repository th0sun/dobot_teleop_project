import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. SETUP PATHS
    # ========================================================================
    pkg_share = get_package_share_directory('mg400_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mg400.urdf.xacro')
    # แปลง Xacro เป็น URDF
    robot_description_content = Command(['xacro ', xacro_file])

    # ========================================================================
    # 2. DEFINE NODES
    # ========================================================================
    
    # [Node 1] Robot State Publisher: คำนวณ TF และรูปร่างหุ่น 3D
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # [Node 2] Joint State Manager: คนกลางที่รับค่าจาก Driver มากระจายต่อ
    node_joint_state_manager = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_manager',
        output='screen',
        parameters=[{
            'source_list': ['/dobot_driver/joint_states'], # รอรับค่าจากข้อ 2
            'rate': 30
        }]
    )

    # [Node 3] RViz2: หน้าจอแสดงผล
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # ========================================================================
    # 3. RETURN LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_manager,
        node_rviz
    ])