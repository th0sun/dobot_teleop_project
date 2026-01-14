import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class TestAnimator(Node):
    def __init__(self):
        super().__init__('test_animator')
        
        # --- Configuration ---
        # ส่งข้อมูลเข้าหัวข้อที่ตัวจัดการ (Bridge Node) รอฟังอยู่
        self.topic_name = '/dobot_driver/joint_states'
        self.frequency = 30.0  # Hz (ความถี่การอัปเดต)

        # --- Publisher Setup ---
        self.publisher_ = self.create_publisher(JointState, self.topic_name, 10)
        
        # --- Timer Setup ---
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info(f'MG400 Animator Started. Publishing to: {self.topic_name}')

    def timer_callback(self):
        # คำนวณเวลาที่ผ่านไป (Time delta)
        current_time = time.time() - self.start_time
        
        # 1. INPUT: จำลองค่ามุมข้อต่อ (Sine Wave Simulation)
        # ในการใช้งานจริง: รับค่าเหล่านี้มาจาก Unity / Controller / Hardware
        target_j1 = 0.5 * math.sin(current_time)
        target_j2 = 0.4 * math.sin(current_time * 0.5)
        target_j3 = 0.4 * math.cos(current_time * 0.5)
        target_j4 = 0.5 * math.cos(current_time)  # ปลายมือ (Servo J5)

        # 2. MESSAGE: เตรียมข้อมูลส่ง
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # ชื่อ Joint ต้องตรงกับไฟล์ URDF เป๊ะๆ
        msg.name = [
            'mg400_j1', 
            'mg400_j2_1', 'mg400_j2_2', 
            'mg400_j3_1', 'mg400_j3_2', 
            'mg400_j4_1', 'mg400_j4_2', 
            'mg400_j5'
        ]

        # 3. KINEMATICS: คำนวณความสัมพันธ์กลไก (Parallel Linkage Logic)
        # สูตรนี้ช่วยให้แขนกลใน RViz ขยับสมจริง ไม่ฉีกออกจากกัน
        msg.position = [
            target_j1,                    # Axis 1: Base
            target_j2,                    # Axis 2: Rear Arm (Main)
            target_j2,                    # Axis 2: Rear Arm (Parallel Link)
            target_j3 - target_j2,        # Axis 3: Forearm (Compensated) *สำคัญ*
            -target_j2,                   # Axis 3: Linkage rod
            -target_j3,                   # Axis 4: Pitch leveling (1)
            target_j3,                    # Axis 4: Pitch leveling (2)
            target_j4                     # Axis 5: End Effector Rotation
        ]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    animator = TestAnimator()
    
    try:
        # Loop การทำงานจนกว่าจะกด Ctrl+C
        rclpy.spin(animator)
        
    except KeyboardInterrupt:
        # ใช้ print แทน get_logger เพื่อเลี่ยง Error ตอน Context ถูกทำลาย
        print("\n[INFO] Keyboard Interrupt detected. Stopping animation...")
        
    finally:
        # Clean Shutdown: ปิด Node และตรวจสอบก่อนปิดระบบ ROS
        animator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[INFO] Shutdown complete.")

if __name__ == '__main__':
    main()