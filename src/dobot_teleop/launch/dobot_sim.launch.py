import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. การตั้งค่า Path ---
    # ระบุตำแหน่งไฟล์โมเดลหุ่นยนต์ (URDF/Xacro)
    pkg_share = get_package_share_directory('mg400_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mg400.urdf.xacro')
    
    # แปลงไฟล์ Xacro เป็น URDF
    robot_description_content = Command(['xacro ', xacro_file])

    # --- 2. การประกาศ Node ---
    
    # Node 1: Robot State Publisher
    # ทำหน้าที่คำนวณตำแหน่ง 3D (TF) ของแขนแต่ละท่อนจาก URDF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Node 2: Joint State Manager
    # ทำหน้าที่รับค่ามุมข้อต่อที่ "ผ่านการ Mix แล้ว" จาก Topic มาแสดงผล
    node_joint_state_manager = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_manager',
        output='screen',
        parameters=[{
            'source_list': ['/dobot_driver/joint_states'], # รอรับค่าจาก Mixer
            'rate': 30
        }]
    )

    # Node 3: RViz2
    # โปรแกรมแสดงผลกราฟิก
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Node 4: Command Processor (Mixer)
    # รันตัวกลางอัตโนมัติ เพื่อรองรับหลาย Controller (Shared Bus)
    node_mixer = Node(
        package='dobot_teleop',
        executable='command_processor.py',
        name='central_mixer',
        output='screen'
    )

    # --- 3. ส่งคืนค่าเพื่อเริ่มทำงาน ---
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_manager,
        node_rviz,
        node_mixer
    ])