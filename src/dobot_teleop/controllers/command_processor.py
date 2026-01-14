#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class CommandProcessor(Node):
    def __init__(self):
        super().__init__('command_processor')
        
        # --- Config ---
        self.sub_topic = '/dobot/cmd_request'       # รับคำสั่งรวม
        self.pub_topic = '/dobot_driver/joint_states' # ส่งไป Driver
        self.smooth_factor = 0.1                    # ความนุ่มนวล (0.1 = หนืดพอดีๆ)
        self.timeout_sec = 0.5                      # เวลา Timeout ตัดการเชื่อมต่อ

        # --- Limits (MG400 Specs) ---
        self.limits = [
            (math.radians(-160), math.radians(160)), # J1
            (math.radians(-25),  math.radians(85)),  # J2
            (math.radians(-25),  math.radians(105)), # J3
            (math.radians(-360), math.radians(360))  # J4
        ]

        # --- Priority Groups ---
        # ถ้า ID มีคำเหล่านี้ จะถือเป็น Manual (Override Auto ได้ทันที)
        self.manual_keywords = ["slider", "joystick", "vr", "manual"]

        # --- ROS Setup ---
        self.publisher_ = self.create_publisher(JointState, self.pub_topic, 10)
        self.create_subscription(JointState, self.sub_topic, self.cmd_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.feedback_callback, 10)

        # Variables
        self.active_sources = {}
        self.current_output = None # เริ่มต้นเป็น None เพื่อรอค่าจริงจากหุ่น
        
        self.create_timer(0.02, self.control_loop) # 50Hz Loop

    def feedback_callback(self, msg):
        """ อ่านค่าเริ่มต้นจากหุ่น เพื่อกันกระตุก (ทำงานครั้งแรกครั้งเดียว) """
        if self.current_output is None:
            try:
                if 'mg400_j1' in msg.name:
                    # Map ค่าจริงเข้าตัวแปรระบบ
                    j1 = msg.position[msg.name.index('mg400_j1')]
                    j2 = msg.position[msg.name.index('mg400_j2_1')]
                    j3 = msg.position[msg.name.index('mg400_j3_1')]
                    j4 = msg.position[msg.name.index('mg400_j5')]
                    self.current_output = [j1, j2, j2, j3, j4]
            except ValueError:
                pass

    def cmd_callback(self, msg):
        """ รับคำสั่งจากทุก Controller และบันทึกเวลา """
        src_id = msg.header.frame_id or "unknown"
        try:
            # แปลง msg เป็น array [j1, j2, j2, j3, j5]
            new_joints = [0.0] * 5
            if 'mg400_j1' in msg.name:
                new_joints[0] = msg.position[msg.name.index('mg400_j1')]
                new_joints[1] = msg.position[msg.name.index('mg400_j2_1')]
                new_joints[2] = msg.position[msg.name.index('mg400_j2_2')]
                new_joints[3] = msg.position[msg.name.index('mg400_j3_1')]
                new_joints[4] = msg.position[msg.name.index('mg400_j5')]

                self.active_sources[src_id] = {
                    'joints': new_joints,
                    'time': self.get_clock().now().nanoseconds / 1e9
                }
        except ValueError:
            pass

    def control_loop(self):
        # รอจนกว่าจะได้ค่าเริ่มต้นจากหุ่น
        if self.current_output is None: return

        now = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Prune: ลบ Controller ที่เงียบไปนานเกิน Timeout
        # (เช่น ปล่อยจอยสติ๊ก หรือปิดโปรแกรม)
        t_out = self.timeout_sec
        active_ids = [sid for sid, d in self.active_sources.items() if (now - d['time']) <= t_out]
        
        # อัปเดต active_sources ให้เหลือแค่ตัวที่ยังอยู่
        self.active_sources = {sid: self.active_sources[sid] for sid in active_ids}

        # 2. Priority: แยกกลุ่ม Manual vs Auto
        manual_cmds = []
        auto_cmds = []
        
        for sid in active_ids:
            joints = self.active_sources[sid]['joints']
            # เช็คชื่อ ID ว่าเข้าข่าย Manual ไหม
            if any(k in sid for k in self.manual_keywords):
                manual_cmds.append(joints)
            else:
                auto_cmds.append(joints)

        # เลือกกลุ่มที่จะใช้ (Manual ชนะขาด)
        target_group = manual_cmds if manual_cmds else auto_cmds
        
        # 3. Calculate Target: หาค่าเฉลี่ยในกลุ่มที่ชนะ
        target = self.current_output[:] # Default: Hold Position
        
        if target_group:
            sum_j = [0.0] * 5
            for j_set in target_group:
                for i in range(5): sum_j[i] += j_set[i]
            target = [x / len(target_group) for x in sum_j]

        # 4. Smooth & Limit & Publish
        self.process_and_publish(target)

    def process_and_publish(self, target):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['mg400_j1', 'mg400_j2_1', 'mg400_j2_2', 'mg400_j3_1', 'mg400_j3_2', 'mg400_j4_1', 'mg400_j4_2', 'mg400_j5']
        
        final_vals = [0.0] * 5
        
        # J1
        final_vals[0] = self.smooth_clamp(0, target[0], 0)
        # J2
        final_vals[1] = self.smooth_clamp(1, target[1], 1)
        # J3 (Index 3)
        final_vals[3] = self.smooth_clamp(3, target[3], 2)
        # J5 (Index 4)
        final_vals[4] = self.smooth_clamp(4, target[4], 3)

        # Update Current & Kinematics
        j1, j2, j3, j4 = final_vals[0], final_vals[1], final_vals[3], final_vals[4]
        j_head = -(j2 + j3)
        
        self.current_output = [j1, j2, j2, j3, j4]
        msg.position = [j1, j2, j2, j3, -j2, j_head, -j_head, j4]
        
        self.publisher_.publish(msg)

    def smooth_clamp(self, idx, target_val, limit_idx):
        # Smooth
        curr = self.current_output[idx]
        new_val = curr + (target_val - curr) * self.smooth_factor
        
        # Clamp
        min_l, max_l = self.limits[limit_idx]
        return max(min(new_val, max_l), min_l)

def main(args=None):
    rclpy.init(args=args)
    node = CommandProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()