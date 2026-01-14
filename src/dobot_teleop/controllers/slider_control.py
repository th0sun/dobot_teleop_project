#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
import math
import tkinter as tk
from threading import Thread

class DobotSliderController(Node):
    def __init__(self):
        super().__init__('dobot_slider_controller')
        
        # Topic ที่จะส่งคำสั่ง
        self.pub_topic = '/dobot/cmd_request'
        self.publisher_ = self.create_publisher(JointState, self.pub_topic, 10)
        
        # ตัวแปรเก็บค่ามุม
        self.target_j1 = 0.0
        self.target_j2 = 0.0
        self.target_j3 = 0.0
        self.target_j4 = 0.0 
        
        self.slider_widgets = {}
        
        # [หัวใจสำคัญ] Set นี้จะเก็บว่า "ตอนนี้เรากำลังจับสไลด์ตัวไหนอยู่บ้าง"
        # ถ้าจับอยู่ -> เราจะไม่ฟังค่าจากหุ่น (เพื่อกันสไลด์สั่นสู้มือ)
        self.active_touch_joints = set()

        self.get_logger().info('Touch-Priority Controller Started.')

        # ฟังค่าจากหุ่น (เพื่ออัปเดตสไลด์ตัวที่เราไม่ได้จับ)
        self.create_subscription(JointState, '/joint_states', self.sync_callback, 10)

    def sync_callback(self, msg):
        """ 
        ฟังก์ชันนี้ทำงานเมื่อหุ่นขยับ (Feedback Loop)
        หน้าที่: อัปเดตหน้าจอให้ตรงกับความจริง *เฉพาะตัวที่มือว่างอยู่*
        """
        try:
            if 'mg400_j1' in msg.name:
                idx_map = {
                    1: msg.name.index('mg400_j1'),
                    2: msg.name.index('mg400_j2_1'),
                    3: msg.name.index('mg400_j3_1'),
                    4: msg.name.index('mg400_j5')
                }

                # วนลูปเช็คทุก Joint
                for j_idx, array_idx in idx_map.items():
                    # แปลงค่าจริงจากหุ่นเป็น Degree
                    real_deg = math.degrees(msg.position[array_idx])

                    # [กฎเหล็ก] เช็คก่อนว่า "เราจับตัวนี้อยู่ไหม?"
                    if j_idx in self.active_touch_joints:
                        # ถ้าจับอยู่: "ช่างหัวหุ่น" (Ignore Feedback) 
                        # เพื่อให้มือเราคุมได้นิ่งๆ ไม่มีการดึงกลับ
                        continue 
                    
                    # ถ้าไม่ได้จับ: "อัปเดตตามเพื่อน" (Sync)
                    if j_idx in self.slider_widgets:
                        self.slider_widgets[j_idx].set(real_deg)
                        # อัปเดตตัวแปรภายในด้วย เดี๋ยวส่งค่าผิด
                        self.update_internal_vars(j_idx, real_deg)

        except ValueError:
            pass

    # --- Mouse Event Handlers (ตัวจัดการเมาส์) ---

    def on_touch_start(self, event, j_idx):
        """ เมื่อเริ่มกดเมาส์: ประกาศตัวว่าเป็น Master ของ Joint นี้ """
        self.active_touch_joints.add(j_idx)

    def on_touch_move(self, event, j_idx):
        """ เมื่อลากเมาส์: ส่งคำสั่งรัวๆ (Last Command Wins) """
        # อ่านค่าจากสไลด์ที่เราลากอยู่
        val = self.slider_widgets[j_idx].get()
        self.update_internal_vars(j_idx, val)
        self.publish_joint_states() # ส่งคำสั่งไป ROS ทันที

    def on_touch_end(self, event, j_idx):
        """ เมื่อปล่อยเมาส์: สละตำแหน่ง Master กลับเป็น Slave """
        # ส่งค่าปิดท้าย 1 ทีเพื่อความชัวร์
        val = self.slider_widgets[j_idx].get()
        self.update_internal_vars(j_idx, val)
        self.publish_joint_states()
        
        # ลบออกจาก Set (กลับไปฟังค่าจากหุ่นตามเดิม)
        self.active_touch_joints.discard(j_idx)

    def update_internal_vars(self, j_idx, val_deg):
        """ แปลง Degree -> Radian เก็บเข้าตัวแปร """
        rad = math.radians(float(val_deg))
        if j_idx == 1: self.target_j1 = rad
        elif j_idx == 2: self.target_j2 = rad
        elif j_idx == 3: self.target_j3 = rad
        elif j_idx == 4: self.target_j4 = rad

    def publish_joint_states(self):
        """ แพ็คข้อมูลส่ง ROS พร้อมระบุตัวตน """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # [สำคัญ!] ระบุ ID ของคอนโทรลเลอร์ตัวนี้ (เพื่อให้ Mixer แยกแยะได้)
        msg.header.frame_id = "slider_controller"
        
        msg.name = ['mg400_j1', 'mg400_j2_1', 'mg400_j2_2', 'mg400_j3_1', 'mg400_j3_2', 'mg400_j4_1', 'mg400_j4_2', 'mg400_j5']
        
        j1, j2, j3, j4 = self.target_j1, self.target_j2, self.target_j3, self.target_j4
        j_head_comp = -(j2 + j3)
        msg.position = [j1, j2, j2, j3, -j2, j_head_comp, -j_head_comp, j4]
        
        self.publisher_.publish(msg)

# --- Thread & GUI ---
def spin_ros_node(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException: pass

def run_gui(node):
    root = tk.Tk()
    root.title("Dobot MG400 (Multi-Master)")
    root.geometry("400x380")

    def create_slider(label, j_idx, min_v, max_v):
        frame = tk.Frame(root, pady=10)
        frame.pack(fill='x', padx=20)
        tk.Label(frame, text=label, font=("Arial", 10, "bold")).pack(anchor='w')
        
        # [สำคัญ] ไม่ใส่ command=... เพื่อป้องกันลูปนรก
        scale = tk.Scale(frame, from_=min_v, to=max_v, orient='horizontal', resolution=0.1)
        scale.pack(fill='x')
        
        # ผูก Event เมาส์ เพื่อจัดการ Priority
        # <Button-1>: เริ่มจับ (Touch Start) -> เลิกฟังหุ่น
        scale.bind("<Button-1>", lambda e, j=j_idx: node.on_touch_start(e, j))
        
        # <B1-Motion>: กำลังลาก (Dragging) -> ส่งคำสั่ง
        scale.bind("<B1-Motion>", lambda e, j=j_idx: node.on_touch_move(e, j))
        
        # <ButtonRelease-1>: ปล่อยมือ (Touch End) -> กลับไปฟังหุ่น
        scale.bind("<ButtonRelease-1>", lambda e, j=j_idx: node.on_touch_end(e, j))

        node.slider_widgets[j_idx] = scale

    create_slider("J1 Base", 1, -160, 160)
    create_slider("J2 Rear", 2, -25, 85)
    create_slider("J3 Front", 3, -25, 105)
    create_slider("J4 Rot", 4, -360, 360)

    tk.Button(root, text="EXIT", bg="red", fg="white", command=root.destroy).pack(pady=20)
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = DobotSliderController()
    Thread(target=spin_ros_node, args=(node,), daemon=True).start()
    try: run_gui(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()