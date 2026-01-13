#!/usr/bin/env python3
"""
Dobot MG400 ROS2 Driver Node
Subscribe to: /dobot/target_pose (dobot_teleop/msg/TargetPose)
Action: Control Robot via MG400Interface
Author: KingProFire
"""

import rclpy
from rclpy.node import Node

# Import Message ที่เราสร้างเอง
from dobot_teleop.msg import TargetPose

# Import Class คุมหุ่นที่เราเขียนเมื่อกี้ (ต้องวางไฟล์ไว้ที่เดียวกัน)
from mg400_interface import MG400Interface

class DobotDriverNode(Node):
    def __init__(self):
        super().__init__('dobot_driver')
        
        # --- 1. Setup Robot Connection ---
        # ใช้ use_dummy=True ไปก่อน เพราะเรายังไม่มีหุ่นจริง
        self.get_logger().info('🤖 Initializing Robot Interface...')
        self.robot = MG400Interface(ip='192.168.1.6', use_dummy=True)
        
        if self.robot.connect():
            self.get_logger().info('✅ Robot Connected Successfully!')
        else:
            self.get_logger().error('❌ Failed to connect to Robot!')
            # ในงานจริง ถ้าต่อไม่ได้อาจจะสั่งปิด Node เลยก็ได้
        
        # --- 2. Create Subscriber (หูฟัง) ---
        # Topic Name: /dobot/target_pose
        # Msg Type: TargetPose
        # Callback: self.motion_callback (เมื่อมีของมา ให้เรียกฟังก์ชันนี้)
        self.subscription = self.create_subscription(
            TargetPose,
            '/dobot/target_pose',
            self.motion_callback,
            10  # Queue Size (เก็บสะสมได้ 10 ข้อความถ้าทำไม่ทัน)
        )
        
        self.get_logger().info('👂 Waiting for commands at /dobot/target_pose ...')

    def motion_callback(self, msg: TargetPose):
        """
        ทำงานทุกครั้งที่มี Message เข้ามา
        :param msg: ข้อมูลที่ส่งมาจาก Topic (TargetPose)
        """
        # --- Safety Check (Critical!) ---
        # ถ้าสวิตช์ Safety (is_enabled) เป็น False ต้องไม่ขยับ
        if not msg.is_enabled:
            # self.get_logger().warn('⛔ Robot is DISABLED via Safety Switch', throttle_duration_sec=2.0)
            return

        # --- Command Execution ---
        # สั่งหุ่นยนต์ผ่าน Interface
        # msg.j1, msg.j2 ... คือชื่อตัวแปรที่เราประกาศในไฟล์ .msg
        self.robot.servo_j(msg.j1, msg.j2, msg.j3, msg.j4)
        
        # (Optional) Log แบบ Debug ถ้าอยากเห็นค่า
        # self.get_logger().debug(f'Moving to: {msg.j1}, {msg.j2}...')

    def on_shutdown(self):
        """สิ่งที่ต้องทำก่อนตาย (Cleanup)"""
        self.get_logger().info('🛑 Shutting down driver...')
        self.robot.disconnect()

def main(args=None):
    # 1. เริ่มระบบ ROS2
    rclpy.init(args=args)
    
    # 2. สร้าง Node
    node = DobotDriverNode()
    
    # 3. วนลูปทำงานไปเรื่อยๆ (Spin) จนกว่าจะกด Ctrl+C
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. จบงานแบบสวยงาม
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
