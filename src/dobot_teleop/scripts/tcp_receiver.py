#!/usr/bin/env python3
"""
TCP Receiver Node (The Translator)
Role: รับ JSON จาก Unity -> แปลงเป็น ROS2 Msg -> ส่งให้ Driver
Author: KingProFire
"""

import rclpy
from rclpy.node import Node
import socket
import threading
import json
import time

# Import Message ของเรา
from dobot_teleop.msg import TargetPose

class TCPReceiverNode(Node):
    def __init__(self):
        super().__init__('tcp_receiver')

        # --- Config ---
        self.HOST = '0.0.0.0'  # รับจากทุกเครื่อง (สำคัญ! ไม่งั้น Unity มองไม่เห็น)
        self.PORT = 5000       # Port ที่นัดกับ Unity ไว้
        self.BUFFER_SIZE = 1024

        # --- Publisher ---
        # ตัวประกาศข่าว สร้าง Topic ชื่อเดียวกับที่ Driver รอฟังอยู่
        self.publisher_ = self.create_publisher(TargetPose, '/dobot/target_pose', 10)

        # --- Start TCP Server Thread ---
        # ต้องแยก Thread เพราะการรอรับข้อมูล (socket.accept) มันจะบล็อกโปรแกรม
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.daemon = True # ถ้าปิดโปรแกรมหลัก Thread นี้จะตายตาม
        self.server_thread.start()

        self.get_logger().info(f'📡 TCP Receiver waiting on {self.HOST}:{self.PORT}')

    def start_server(self):
        """ฟังก์ชันเฝ้าประตู (รันใน Thread แยก)"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # อนุญาตให้ใช้ Port ซ้ำได้ทันที (กัน Error: Address already in use)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            try:
                s.bind((self.HOST, self.PORT))
                s.listen()
            except Exception as e:
                self.get_logger().error(f'❌ Failed to bind port: {e}')
                return

            while rclpy.ok():
                self.get_logger().info('⏳ Waiting for Unity to connect...')
                conn, addr = s.accept() # บรรทัดนี้จะหยุดรอจนกว่า Unity จะต่อมา
                
                with conn:
                    self.get_logger().info(f'✅ Connected by {addr}')
                    self.handle_client(conn)
                
                self.get_logger().warn('⚠️ Unity disconnected. Re-listening...')

    def handle_client(self, conn):
        """ฟังก์ชันคุยกับแขก (Unity)"""
        while rclpy.ok():
            try:
                data = conn.recv(self.BUFFER_SIZE)
                if not data:
                    break # ถ้าส่งมาว่างเปล่า แปลว่าหลุด
                
                # 1. แปลง Byte เป็น String (JSON)
                json_str = data.decode('utf-8').strip()
                # self.get_logger().info(f'Raw Data: {json_str}') # Debug ดูได้

                # 2. แกะกล่อง JSON
                try:
                    data_dict = json.loads(json_str)
                    
                    # 3. ยัดใส่ ROS2 Message
                    msg = TargetPose()
                    msg.j1 = float(data_dict.get('j1', 0.0))
                    msg.j2 = float(data_dict.get('j2', 0.0))
                    msg.j3 = float(data_dict.get('j3', 0.0))
                    msg.j4 = float(data_dict.get('j4', 0.0))
                    msg.gripper_state = int(data_dict.get('gripper', 0))
                    msg.is_enabled = bool(data_dict.get('enabled', False))

                    # 4. ส่งต่อให้ Driver (Publish)
                    self.publisher_.publish(msg)

                except json.JSONDecodeError:
                    self.get_logger().warn(f'🗑️ Bad JSON received: {json_str}')
                except ValueError as e:
                    self.get_logger().warn(f'🔢 Data type error: {e}')

            except Exception as e:
                self.get_logger().error(f'❌ Connection Error: {e}')
                break

def main(args=None):
    rclpy.init(args=args)
    node = TCPReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
