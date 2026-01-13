#!/usr/bin/env python3
"""
MG400 TCP/IP Interface Driver
Standard: Python 3.8+
Author: KingProFire
"""

import socket
import time

# --- Configuration Constants (ค่าคงที่) ---
ROBOT_REAL_PORT = 30003   # Port สำหรับสั่งเคลื่อนที่ (Motion Port)
DASHBOARD_PORT = 29999    # Port สำหรับสั่ง Enable/Reset (Dashboard)
SOCKET_TIMEOUT = 5.0      # รอเชื่อมต่อสูงสุด 5 วินาที

class MG400Interface:
    def __init__(self, ip: str = '192.168.1.6', use_dummy: bool = False):
        """
        ตัวจัดการการเชื่อมต่อหุ่นยนต์ Dobot MG400
        :param ip: IP Address ของหุ่นยนต์
        :param use_dummy: ถ้า True จะเป็นโหมดจำลอง (ไม่ต่อหุ่นจริง)
        """
        self.ip = ip
        self.port = ROBOT_REAL_PORT
        self.use_dummy = use_dummy
        
        self.socket = None
        self.is_connected = False
        
        if self.use_dummy:
            print(f"⚠️  [INIT] Running in DUMMY MODE (Virtual Robot)")

    def connect(self) -> bool:
        """เริ่มการเชื่อมต่อ TCP Socket"""
        if self.use_dummy:
            self.is_connected = True
            print(f"✅ [DUMMY] Connected to virtual robot at {self.ip}")
            return True

        try:
            print(f"🔌 Connecting to {self.ip}:{self.port}...")
            self.socket = socket.socket(socket.AF_INET, socket.socket.SOCK_STREAM)
            self.socket.settimeout(SOCKET_TIMEOUT)
            self.socket.connect((self.ip, self.port))
            
            self.is_connected = True
            print(f"✅ Connected to Dobot MG400!")
            return True
            
        except OSError as e:
            print(f"❌ Connection Error: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """ตัดการเชื่อมต่อและคืนทรัพยากร"""
        if self.use_dummy:
            self.is_connected = False
            print("🔌 [DUMMY] Disconnected.")
            return

        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except Exception as e:
                print(f"⚠️ Error closing socket: {e}")
        
        self.is_connected = False
        print("🔌 Disconnected.")

    def servo_j(self, j1: float, j2: float, j3: float, j4: float):
        """
        ส่งคำสั่ง ServoJ (Real-time Joint Control)
        :param j1-j4: มุมของแต่ละข้อต่อ (องศา)
        """
        # สร้างคำสั่งตามคู่มือ Dobot: ServoJ(j1, j2, j3, j4)
        # \n คือการกด Enter เพื่อส่งคำสั่ง
        command_str = f"ServoJ({j1:.3f},{j2:.3f},{j3:.3f},{j4:.3f})\n"
        
        self._send(command_str)

    def _send(self, data: str):
        """
        (Private Method) ฟังก์ชันส่งข้อมูลดิบผ่าน Socket
        คนภายนอกไม่ควรเรียกใช้ฟังก์ชันนี้ตรงๆ
        """
        if not self.is_connected:
            print("⚠️ Cannot send: Robot not connected")
            return

        # กรณี Dummy Mode: แค่พริ้นต์หลอกๆ
        if self.use_dummy:
            # end='\r' ทำให้ข้อความทับบรรทัดเดิม (เหมือนโหลดดิ้งบาร์)
            print(f"🚀 [DUMMY SEND]: {data.strip()}      ", end='\r')
            return

        # กรณี Real Mode: ส่งจริง
        try:
            self.socket.sendall(data.encode('utf-8'))
        except Exception as e:
            print(f"❌ Send Failed: {e}")
            self.is_connected = False

# --- Unit Test Section ---
# ส่วนนี้จะทำงานเฉพาะตอนเรารันไฟล์นี้ตรงๆ (เอาไว้เทส)
# แต่ถ้าไฟล์นี้ถูก import ไปใช้ที่อื่น ส่วนนี้จะไม่ทำงาน
if __name__ == "__main__":
    # ลองเทสแบบ Dummy (ไม่ต้องแก้ IP ก็รันได้)
    robot = MG400Interface(ip='192.168.1.6', use_dummy=True)

    if robot.connect():
        try:
            print("\n🏁 Starting Test Loop (Press Ctrl+C to stop)...")
            val = 0.0
            step = 0.5
            
            # วนลูปส่งค่าเล่นๆ เพื่อดูว่ามันทำงานไหม
            while True:
                # ลองขยับ J1 ไป-กลับ ระหว่าง -30 ถึง 30
                if val >= 30 or val <= -30:
                    step *= -1
                val += step
                
                robot.servo_j(val, 0.0, 0.0, 0.0)
                
                # จำลองความถี่ 50Hz (ส่งทุกๆ 0.02 วินาที)
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping test...")
        finally:
            robot.disconnect()
