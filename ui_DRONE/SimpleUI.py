import sys
import json
import time
import glob
import serial
import serial.tools.list_ports
import datetime
import numpy as np
import pyqtgraph as pg 

from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QGridLayout, QLabel, QComboBox, 
                               QPushButton, QLineEdit, QGroupBox, QMessageBox)
from PySide6.QtCore import QThread, Signal, Slot, Qt

# --- CẤU HÌNH ---
DEFAULT_PORT = '/dev/ttyAMA0'
DEFAULT_BAUD = 115200
LOG_FILE_NAME = "pid_history.txt"

# Độ dài cửa sổ hiển thị (Số điểm mẫu)
# Nếu STM32 gửi 50Hz (20ms/gói) -> 500 điểm = 10 giây lịch sử
GRAPH_BUFFER_SIZE = 500 

# ============================================================================
# LUỒNG ĐỌC SERIAL
# ============================================================================
class SerialWorker(QThread):
    telemetry_signal = Signal(dict)
    log_signal = Signal(str)
    status_signal = Signal(bool)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_running = False
        self.port = ""
        self.baud = 115200

    def connect_serial(self, port, baud):
        self.port = port
        self.baud = baud
        self.start()

    def disconnect_serial(self):
        self.is_running = False
        self.wait()

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.ser.flush()
            except Exception as e:
                self.log_signal.emit(f"❌ Lỗi gửi: {e}")

    def run(self):
        try:
            if self.ser and self.ser.is_open: self.ser.close()
            # Timeout cực ngắn để đọc non-blocking
            self.ser = serial.Serial(self.port, self.baud, timeout=0.005)
            self.is_running = True
            self.status_signal.emit(True)
            self.log_signal.emit(f"✅ Đã kết nối {self.port}")

            buffer = ""
            while self.is_running and self.ser.is_open:
                try:
                    if self.ser.in_waiting:
                        raw = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        buffer += raw
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if not line: continue

                            if line.startswith('{') and line.endswith('}'):
                                try:
                                    data = json.loads(line)
                                    self.telemetry_signal.emit(data)
                                except: pass
                            else:
                                self.log_signal.emit(f"STM32: {line}")
                    else:
                        time.sleep(0.001) 
                except Exception as e:
                    self.log_signal.emit(f"❌ Lỗi đọc: {e}")
                    time.sleep(1)
        except Exception as e:
            self.log_signal.emit(f"❌ Lỗi mở cổng: {e}")
            self.status_signal.emit(False)
        finally:
            if self.ser: self.ser.close()
            self.status_signal.emit(False)

# ============================================================================
# GIAO DIỆN CHÍNH
# ============================================================================
class RealtimeTunerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Realtime PID Tuner (Time Domain)")
        self.resize(1000, 900)

        # Mảng dữ liệu
        self.data_time = np.zeros(GRAPH_BUFFER_SIZE) # Trục thời gian
        self.data_roll = np.zeros(GRAPH_BUFFER_SIZE)
        self.data_pitch = np.zeros(GRAPH_BUFFER_SIZE)
        self.data_yaw = np.zeros(GRAPH_BUFFER_SIZE)
        
        self.start_time = 0 # Mốc thời gian bắt đầu

        self.worker = SerialWorker()
        self.worker.telemetry_signal.connect(self.update_telemetry)
        self.worker.log_signal.connect(self.update_log)
        self.worker.status_signal.connect(self.update_connection_status)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout(central_widget)
        self.main_layout.setSpacing(5)
        self.main_layout.setContentsMargins(5,5,5,5)

        self.setup_connection_ui()
        self.setup_telemetry_text_ui()
        self.setup_realtime_graph()
        self.setup_tuning_ui()
        
        self.lbl_status = QLabel("Sẵn sàng.")
        self.lbl_status.setStyleSheet("color: #666; font-style: italic;")
        self.main_layout.addWidget(self.lbl_status)

        self.refresh_ports()

    def setup_connection_ui(self):
        group = QGroupBox("KẾT NỐI")
        group.setStyleSheet("font-weight: bold;")
        layout = QHBoxLayout()
        layout.setContentsMargins(5,5,5,5)

        self.cbo_ports = QComboBox()
        btn_refresh = QPushButton("Ref")
        btn_refresh.setFixedWidth(40)
        btn_refresh.clicked.connect(self.refresh_ports)

        self.txt_baud = QLineEdit(str(DEFAULT_BAUD))
        self.txt_baud.setFixedWidth(70)
        self.txt_baud.setAlignment(Qt.AlignCenter)

        self.btn_connect = QPushButton("KẾT NỐI")
        self.btn_connect.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
        self.btn_connect.clicked.connect(self.toggle_connection)

        layout.addWidget(self.cbo_ports, 1)
        layout.addWidget(btn_refresh)
        layout.addWidget(self.txt_baud)
        layout.addWidget(self.btn_connect, 1)
        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def setup_telemetry_text_ui(self):
        group = QGroupBox()
        layout = QGridLayout()
        layout.setContentsMargins(0,0,0,0)

        def create_lbl(color):
            l = QLabel("0.0°")
            l.setAlignment(Qt.AlignCenter)
            l.setStyleSheet(f"font-size: 24px; font-weight: bold; color: {color};")
            return l

        layout.addWidget(QLabel("ROLL"), 0, 0, Qt.AlignCenter)
        self.val_roll = create_lbl("#FF3333") 
        layout.addWidget(self.val_roll, 1, 0)

        layout.addWidget(QLabel("PITCH"), 0, 1, Qt.AlignCenter)
        self.val_pitch = create_lbl("#33FF33") 
        layout.addWidget(self.val_pitch, 1, 1)

        layout.addWidget(QLabel("YAW"), 0, 2, Qt.AlignCenter)
        self.val_yaw = create_lbl("#3388FF") 
        layout.addWidget(self.val_yaw, 1, 2)

        self.val_bat = QLabel("0.0 V")
        self.val_bat.setStyleSheet("font-size: 16px; font-weight: bold; color: #555;")
        layout.addWidget(self.val_bat, 0, 3, 2, 1, Qt.AlignCenter)

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def setup_realtime_graph(self):
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#121212')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setYRange(-45, 45)
        
        # --- CẤU HÌNH TRỤC THỜI GIAN ---
        self.plot_widget.setTitle("ĐÁP ỨNG HỆ THỐNG (TIME DOMAIN)", color="#FFF", size="11pt")
        
        label_style = {'color': '#EEE', 'font-size': '10pt', 'font-weight': 'bold'}
        self.plot_widget.setLabel('left', 'Biên độ góc (Độ)', **label_style)
        
        # QUAN TRỌNG: Đổi nhãn thành Giây
        self.plot_widget.setLabel('bottom', 'Thời gian (Giây)', **label_style)
        
        self.plot_widget.addLegend(offset=(10, 10))

        # Khởi tạo đường vẽ
        self.curve_roll = self.plot_widget.plot(name="Roll", pen=pg.mkPen(color='#FF3333', width=2))
        self.curve_pitch = self.plot_widget.plot(name="Pitch", pen=pg.mkPen(color='#33FF33', width=2))
        self.curve_yaw = self.plot_widget.plot(name="Yaw", pen=pg.mkPen(color='#3388FF', width=2))

        self.main_layout.addWidget(self.plot_widget, stretch=1)

    def setup_tuning_ui(self):
        self.pid_layout = QVBoxLayout()
        self.main_layout.addLayout(self.pid_layout)

        self.add_pid_row("1. ANG: Roll & Pitch (Outer Loop)", "ANG", 4.0, 0.0, 0.0)
        self.add_pid_row("2. ANG: Yaw Heading (Outer Loop)", "YANG", 3.0, 0.0, 0.0)
        self.add_pid_row("3. RATE: Yaw Rate (Inner Loop)", "YAW", 1.456, 1.401, 0.138)

    def add_pid_row(self, title, key, p_def, i_def, d_def):
        group = QGroupBox(title)
        group.setStyleSheet("QGroupBox { font-weight: bold; color: #333; border: 1px solid #CCC; margin-top: 5px;} QGroupBox::title { color: #0056b3; }")
        layout = QHBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        input_style = "padding: 3px; border: 1px solid #CCC; font-weight: bold;"

        layout.addWidget(QLabel("P:"))
        txt_p = QLineEdit(str(p_def))
        txt_p.setStyleSheet(input_style)
        layout.addWidget(txt_p)

        layout.addWidget(QLabel("I:"))
        txt_i = QLineEdit(str(i_def))
        txt_i.setStyleSheet(input_style)
        layout.addWidget(txt_i)

        layout.addWidget(QLabel("D:"))
        txt_d = QLineEdit(str(d_def))
        txt_d.setStyleSheet(input_style)
        layout.addWidget(txt_d)

        btn_send = QPushButton("GỬI")
        btn_send.setFixedSize(60, 30)
        btn_send.setStyleSheet("background-color: #17a2b8; color: white; font-weight: bold;")
        btn_send.clicked.connect(lambda: self.send_pid(key, txt_p.text(), txt_i.text(), txt_d.text()))
        
        layout.addWidget(btn_send)
        group.setLayout(layout)
        self.pid_layout.addWidget(group)

    # --- LOGIC ---
    def refresh_ports(self):
        self.cbo_ports.clear()
        ports = serial.tools.list_ports.comports()
        found = [p.device for p in ports]
        pi_uarts = ['/dev/ttyAMA0', '/dev/serial0']
        for p in pi_uarts:
            if glob.glob(p) and p not in found: found.insert(0, p)
        self.cbo_ports.addItems(found)

    def toggle_connection(self):
        if not self.worker.isRunning():
            port = self.cbo_ports.currentText()
            if not port: return
            
            # Reset thời gian về 0 khi bắt đầu kết nối
            self.start_time = time.time()
            
            # Reset mảng dữ liệu về 0 để đồ thị sạch sẽ
            self.data_time.fill(0)
            self.data_roll.fill(0)
            self.data_pitch.fill(0)
            self.data_yaw.fill(0)

            self.worker.connect_serial(port, int(self.txt_baud.text()))
        else:
            self.worker.disconnect_serial()

    @Slot(bool)
    def update_connection_status(self, connected):
        if connected:
            self.btn_connect.setText("NGẮT")
            self.btn_connect.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold;")
            self.cbo_ports.setEnabled(False)
        else:
            self.btn_connect.setText("KẾT NỐI")
            self.btn_connect.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
            self.cbo_ports.setEnabled(True)

    def send_pid(self, axis, p, i, d):
        if not self.worker.isRunning():
            QMessageBox.warning(self, "Lỗi", "Chưa kết nối UART!")
            return
        cmd = f"PID:{axis}:{p}:{i}:{d}\n"
        self.worker.send_command(cmd)
        self.lbl_status.setText(f"Đã gửi: {cmd.strip()}")
        self.write_log_to_file(axis, p, i, d)

    def write_log_to_file(self, axis, p, i, d):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log = f"[{timestamp}] {axis} -> P:{p} I:{i} D:{d}\n"
        try:
            with open(LOG_FILE_NAME, "a", encoding="utf-8") as f: f.write(log)
        except: pass

    @Slot(dict)
    def update_telemetry(self, data):
        # 1. Tính thời gian thực (Giây)
        current_t = time.time() - self.start_time
        
        r = data.get('roll', 0)
        p = data.get('pitch', 0)
        y = data.get('yaw', 0)
        v = data.get('v', 0)

        self.val_roll.setText(f"{r:.1f}°")
        self.val_pitch.setText(f"{p:.1f}°")
        self.val_yaw.setText(f"{y:.1f}°")
        self.val_bat.setText(f"{v:.1f} V")

        # 2. Cập nhật mảng dữ liệu (Shift left)
        self.data_time = np.roll(self.data_time, -1); self.data_time[-1] = current_t
        self.data_roll = np.roll(self.data_roll, -1); self.data_roll[-1] = r
        self.data_pitch = np.roll(self.data_pitch, -1); self.data_pitch[-1] = p
        self.data_yaw = np.roll(self.data_yaw, -1); self.data_yaw[-1] = y

        # 3. Vẽ lại đồ thị với trục X là thời gian
        self.curve_roll.setData(self.data_time, self.data_roll)
        self.curve_pitch.setData(self.data_time, self.data_pitch)
        self.curve_yaw.setData(self.data_time, self.data_yaw)

    @Slot(str)
    def update_log(self, msg):
        self.lbl_status.setText(msg)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RealtimeTunerWindow()
    window.show()
    sys.exit(app.exec())