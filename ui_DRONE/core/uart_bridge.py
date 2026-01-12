import serial
import time
import json  # <--- [QUAN TRỌNG] Thêm thư viện này để đọc JSON từ STM32
from PySide6.QtCore import QThread, Signal

class UARTBridge(QThread):
    # Signal gửi dữ liệu telemetry (Roll, Pitch, Yaw, Battery) về GUI
    data_received = Signal(dict)

    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        
        # Đổi tên thành self.serial để khớp chuẩn với Giao diện
        self.serial = None 

    def run(self):
        while self.running:
            # 1. Cơ chế tự động kết nối lại
            if self.serial is None:
                try:
                    self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
                    print(f"✅ UART BRIDGE: Đã kết nối tại {self.port}")
                except Exception as e:
                    print(f"⚠️ UART BRIDGE: Lỗi kết nối {e}. Thử lại sau 2s...")
                    time.sleep(2)
                    continue

            # 2. Đọc dữ liệu
            try:
                if self.serial.in_waiting:
                    # Đọc 1 dòng từ STM32 (Dạng JSON: {"roll":1.2, "pitch":...})
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        # [DEBUG] Uncomment dòng dưới nếu muốn xem raw data
                        # print(f"Raw: {line}") 
                        
                        data = self.parse_data(line)
                        if data:
                            self.data_received.emit(data)
                            
            except Exception as e:
                print(f"❌ UART BRIDGE: Mất kết nối! ({e})")
                if self.serial:
                    self.serial.close()
                self.serial = None
                time.sleep(1)
   def send_control_command(self, throttle, pitch, roll, yaw):

        if self.serial and self.serial.is_open:
            command = f"CTRL:{int(throttle)}:{int(pitch)}:{int(roll)}:{int(yaw)}\n"
            try:
                self.serial.write(command.encode('utf-8'))
                # print(f"[UART TX] Sent: {command.strip()}")
            except Exception as e:
                print(f"❌ Lỗi gửi UART: {e}")
        else:
            print("⚠️ Serial chưa sẵn sàng để gửi lệnh!")

    def parse_data(self, line):
        """
        Phân tích chuỗi JSON từ STM32.
        Input: '{"roll":0.5,"pitch":1.2,"yaw":10.0,"v":12.6}'
        """
        try:
            # Nếu dòng dữ liệu bắt đầu bằng '{' và kết thúc bằng '}' thì mới parse
            if line.startswith('{') and line.endswith('}'):
                data = json.loads(line)
                return data # Trả về thẳng Dictionary cho GUI
            return None
        except json.JSONDecodeError:
            return None
        except Exception:
            return None

    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()