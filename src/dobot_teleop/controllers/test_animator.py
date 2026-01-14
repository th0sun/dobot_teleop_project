#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class TestAnimator(Node):
    """
    Node สำหรับทดสอบการเคลื่อนที่ของ Dobot MG400 ใน RViz
    โดยการสร้างคลื่น Sine Wave ส่งไปยัง Joint ต่างๆ เพื่อตรวจสอบกลไก (Kinematics)
    """
    def __init__(self):
        super().__init__('test_animator')
        
        # --- 1. CONFIGURATION ---
        self.topic_name = '/dobot/cmd_request' # หัวข้อที่ Bridge Node รอรับ
        self.frequency = 30.0                          # ความถี่การส่งข้อมูล (Hz)

        # --- 2. SETUP PUBLISHER ---
        self.publisher_ = self.create_publisher(JointState, self.topic_name, 10)
        
        # --- 3. SETUP TIMER ---
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info(f'MG400 Animator Started (Datasheet Mode). Topic: {self.topic_name}')

    def timer_callback(self):
        # คำนวณเวลาที่ผ่านไป (สำหรับการสร้างคลื่น Sine)
        current_time = time.time() - self.start_time
        
        # =========================================================================
        # PART A: INPUT GENERATION (จำลองค่าจาก Controller/Trajectory)
        # =========================================================================
        # สร้างการเคลื่อนที่วนลูปด้วย Sine/Cosine
        # target_jX คือค่ามุมที่ "มนุษย์ต้องการสั่ง" (Relative Angle ตาม Datasheet)
        
        # J1 Base: หมุนซ้ายขวา
        target_j1 = 0.5 * math.sin(current_time)
        
        # J2 Rear Arm: โยกหน้าหลัง
        target_j2 = 0.4 * math.sin(current_time * 0.5) 
        
        # J3 Forearm: ยืดหดแขน (ใช้ Cos เพื่อให้จังหวะต่างกับ J2 เล็กน้อย)
        target_j3 = 0.6 * math.cos(current_time * 0.5) 
        
        # J4 Rotation: หมุนปลายมือ
        target_j4 = 0.5 * math.cos(current_time)

        # =========================================================================
        # PART B: KINEMATICS CALCULATION (สูตรคำนวณกลไกขนาน)
        # =========================================================================
        # Dobot MG400 ใช้กลไก Parallel Linkage เพื่อรักษาระนาบ
        # เราต้องแปลงค่า Input เป็นค่าของ Joint จริงๆ ใน Simulation
        
        # สูตรชดเชยหัวหุ่น (Head Leveling):
        # เพื่อให้หน้าแปลนปลายมือ "ขนานกับพื้นเสมอ" (Parallel to Ground)
        # มุมชดเชย = -(มุมแขนท่อนบน + มุมแขนท่อนล่าง)
        head_compensation = -(target_j2 + target_j3)

        # =========================================================================
        # PART C: CONSTRUCT MESSAGE
        # =========================================================================
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sine_wave_animator"
        
        # รายชื่อ Joint ต้องตรงกับไฟล์ URDF ทุกตัวอักษร
        msg.name = [
            'mg400_j1', 
            'mg400_j2_1', 'mg400_j2_2', 
            'mg400_j3_1', 'mg400_j3_2', 
            'mg400_j4_1', 'mg400_j4_2', 
            'mg400_j5'
        ]

        # กำหนดค่าตำแหน่ง (Position)
        msg.position = [
            target_j1,              # Axis 1: ฐานหมุน
            target_j2,              # Axis 2: แขนหลัก (Main Arm)
            target_j2,              # Axis 2: ก้านขนาน (Parallel Link) วิ่งตาม J2
            target_j3,              # Axis 3: แขนหน้า (Forearm) *สูตรใหม่: ใช้ค่าตรงๆ เพื่อให้ยืดสุด*
            -target_j2,             # Axis 3: ก้านดึงหลัง (Linkage Rod) ต้องสวนทางกับ J2
            head_compensation,      # Axis 4: ปรับระนาบหัว (Pitch 1)
            -head_compensation,     # Axis 4: ปรับระนาบหัว (Pitch 2)
            target_j4               # Axis 5: หมุนปลายมือ (End Effector Rotation)
        ]
        
        # ส่งข้อมูลออกไป
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    animator = TestAnimator()
    
    try:
        # รัน Loop จนกว่าจะกด Ctrl+C
        rclpy.spin(animator)
        
    except KeyboardInterrupt:
        # Handle การกด Ctrl+C ให้แสดงข้อความแจ้งเตือนปกติ แทนที่จะขึ้น Error
        print("\n[INFO] Keyboard Interrupt detected. Stopping animation...")
        
    finally:
        # Clean Shutdown: ขั้นตอนการปิดระบบที่ถูกต้อง
        animator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[INFO] Shutdown complete.")

if __name__ == '__main__':
    main()