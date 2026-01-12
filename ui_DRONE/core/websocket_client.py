import json
import time
import websocket # Yêu cầu cài thư viện: pip install websocket-client
from PySide6.QtCore import QThread, Signal

class WSClient(QThread):
    # Signal bắn dữ liệu về UI
    data_received = Signal(dict)
    
    # Signal báo kết nối thành công (để cập nhật trạng thái UI)
    connection_success = Signal()
    
    def __init__(self, url="ws://127.0.0.1:8765"):
        super().__init__()
        self.url = url
        self.ws = None
        self.running = True

    def run(self):
        """
        Chạy trong luồng riêng (Background Thread)
        """
        while self.running:
            try:
                print(f"Connecting to {self.url}...")
                
                # Tạo kết nối với Ping Interval để giữ mạng không bị rớt
                # ping_interval=20: Cứ 20s gửi ping 1 lần
                # ping_timeout=10: Nếu sau 10s không thấy server trả lời -> coi như mất mạng
                self.ws = websocket.create_connection(
                    self.url, 
                    timeout=5, 
                    ping_interval=20, 
                    ping_timeout=10
                )
                
                print("Connected!")
                self.connection_success.emit()
                
                # Vòng lặp nhận dữ liệu
                while self.running:
                    # .recv() sẽ block thread này cho đến khi có tin nhắn
                    message = self.ws.recv()
                    if message:
                        try:
                            data = json.loads(message)
                            self.data_received.emit(data)
                        except json.JSONDecodeError:
                            pass
                            
            except Exception as e:
                print(f"WS Connection Error: {e}. Retrying in 2s...")
                self.ws = None
                time.sleep(2) # Nghỉ 2 giây trước khi thử kết nối lại
            
            finally:
                if self.ws:
                    try:
                        self.ws.close()
                    except:
                        pass
                    self.ws = None

    # ==========================================================
    # CÁC HÀM GỬI LỆNH (SEND COMMANDS)
    # ==========================================================

    def _send(self, payload_dict):
        """Hàm nội bộ để gửi JSON an toàn"""
        if self.ws:
            try:
                payload = json.dumps(payload_dict)
                self.ws.send(payload)
            except Exception as e:
                print(f"Send Error: {e}")

    def send_pid(self, axis, kp, ki, kd):
        """Gửi thông số PID (Roll/Pitch/Yaw)"""
        self._send({
            "type": "set_pid",
            "axis": axis,
            "kp": kp,
            "ki": ki,
            "kd": kd
        })

    def send_motor(self, motor_id, throttle):
        """Gửi lệnh test từng motor (0-100%)"""
        self._send({
            "type": "test_motor",
            "motor_id": motor_id,
            "value": throttle
        })

    def send_throttle(self, value):
        """Gửi giá trị ga nền (Base Throttle) từ Slider"""
        self._send({
            "type": "set_throttle",
            "value": int(value)
        })

    def send_command(self, cmd_type):
        """Gửi lệnh hệ thống (START_PID / STOP_PID)"""
        self._send({
            "type": cmd_type
        })

    def stop(self):
        """Dừng luồng và ngắt kết nối"""
        self.running = False
        if self.ws:
            try:
                self.ws.close()
            except:
                pass