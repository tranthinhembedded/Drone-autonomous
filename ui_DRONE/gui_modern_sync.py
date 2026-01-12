# gui_modern_sync.py
import sys
import json
import time
from PySide6.QtWidgets import (QApplication, QWidget, QHBoxLayout, QVBoxLayout,
                               QGroupBox, QLabel, QFileDialog, QComboBox, QMessageBox)
from PySide6.QtCore import Qt, Slot, QTimer

# ============================================================================
#                               CẤU HÌNH HỆ THỐNG
# ============================================================================
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

STABILIZE_TIME_MS = 4000
IDLE_THROTTLE = 1000      # Ga tắt máy (Disarm)
MIN_SPIN_THROTTLE = 1140  # Ga quay nhẹ (Idle Speed) - Để giữ thăng bằng

try:
    from core.style import apply_modern_style
    from core.uart_bridge import UARTBridge
    from widgets.drone_3d import Drone3D
    from widgets.camera_view import CameraView
    from widgets.tabs import ControlTabs
except ImportError as e:
    print("❌ LỖI IMPORT: Không tìm thấy module 'core' hoặc 'widgets'.")
    sys.exit(1)

class MainPiGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pi Drone Controller - SYNCED SLIDER")
        self.showMaximized()

        self.ready_to_send = False

        main_layout = QHBoxLayout(self)

        # ====================================================================
        #                           CỘT TRÁI (CAMERA + 3D)
        # ====================================================================
        left_layout = QVBoxLayout()
        self.camera_view = CameraView()
        left_layout.addWidget(self.camera_view, stretch=5)

        drone_grp = QGroupBox("TELEMETRY")
        drone_layout = QVBoxLayout()
        self.drone_3d = Drone3D()
        self.drone_3d.setMinimumHeight(300)

        self.lbl_telemetry = QLabel("DISCONNECTED")
        self.lbl_telemetry.setAlignment(Qt.AlignCenter)
        self.lbl_telemetry.setStyleSheet("font-size:18px; color:#555; font-weight:bold;")

        drone_layout.addWidget(self.drone_3d)
        drone_layout.addWidget(self.lbl_telemetry)
        drone_grp.setLayout(drone_layout)
        left_layout.addWidget(drone_grp, stretch=4)
        main_layout.addLayout(left_layout, stretch=6)

        # ====================================================================
        #                           CỘT PHẢI (ĐIỀU KHIỂN)
        # ====================================================================
        right_layout = QVBoxLayout()

        lbl_mode = QLabel("CHỌN CHẾ ĐỘ HOẠT ĐỘNG:")
        lbl_mode.setStyleSheet("font-weight:bold; color:#CCC;")
        right_layout.addWidget(lbl_mode)

        self.combo_mode = QComboBox()
        self.combo_mode.addItems([
            "🛠️ MODE 1: TUNING PID (Tắt Camera - CPU Nhẹ)",
            "✈️ MODE 2: AUTO FLIGHT (Bật Camera - Xử lý ảnh)"
        ])
        self.combo_mode.setStyleSheet("""
            QComboBox { padding: 10px; font-size: 14px; font-weight: bold; background: #222; color: #FFF; border: 2px solid #555; }
            QComboBox::drop-down { border: 0px; }
        """)
        self.combo_mode.currentIndexChanged.connect(self.change_mode)
        right_layout.addWidget(self.combo_mode)

        self.tabs = ControlTabs()
        right_layout.addWidget(self.tabs)
        main_layout.addLayout(right_layout, stretch=4)

        # ====================================================================
        #                           LOGIC KẾT NỐI
        # ====================================================================
        print(f"--> Đang khởi tạo UART tại {SERIAL_PORT}...")
        self.uart = UARTBridge(port=SERIAL_PORT, baudrate=BAUD_RATE)
        self.uart.data_received.connect(self.on_uart_data)
        self.uart.start()

        self.stabilize_timer = QTimer()
        self.stabilize_timer.setSingleShot(True)
        self.stabilize_timer.timeout.connect(self.on_stabilize_finished)

        QTimer.singleShot(2000, self.enable_sending)

        self.camera_view.thread.control_signal.connect(self.process_vision_control)
        self.tabs.vision_param_signal.connect(self.camera_view.thread.update_params)
        self.tabs.request_open_file.connect(self.open_video_file)
        self.tabs.request_camera.connect(self.open_live_camera)

        self.tabs.slider_th.valueChanged.connect(self.manual_control_update)
        self.tabs.btn_arm.clicked.connect(self.toggle_arm)
        self.tabs.btn_start_fly.clicked.connect(self.start_mission)

        self.tabs.motor_tab.send_motor_command.connect(self.send_motor_test)
        self.tabs.pid_update_signal.connect(self.send_pid_to_uart)

        # NEW: REC button connect + thread signal connect
        self.tabs.request_toggle_record.connect(self.toggle_record_video)
        self.camera_view.thread.recording_state_signal.connect(self.on_recording_state_changed)

        self.armed = False
        self.is_stabilizing = False
        self.can_fly = False
        self.current_throttle = 1000
        self.is_vision_mode = False

        # init REC button state
        self.tabs.set_rec_button_state(False)

    def enable_sending(self):
        self.ready_to_send = True
        print("✅ HỆ THỐNG SẴN SÀNG.")
        self.change_mode(0)

    def change_mode(self, index):
        # Khi đổi mode: nếu đang REC thì dừng trước để file không hỏng
        if self.camera_view.thread.is_recording():
            self.camera_view.thread.request_toggle_recording()

        if index == 0:
            print(">>> MODE TUNING: Tắt Camera.")
            self.is_vision_mode = False
            self.camera_view.close_camera()
            self.tabs.btn_start_fly.setEnabled(False)
            self.tabs.btn_start_fly.setText("CHẾ ĐỘ TUNING (NO FLY)")
            self.tabs.btn_start_fly.setStyleSheet("background:#333; color:#777;")

            # REC button về OFF (UI)
            self.tabs.set_rec_button_state(False)
        else:
            print(">>> MODE FLIGHT: Bật Camera.")
            self.is_vision_mode = True
            self.camera_view.thread.set_source(0)
            if self.armed and not self.is_stabilizing:
                self.tabs.btn_start_fly.setEnabled(True)
                self.tabs.btn_start_fly.setText("BẮT ĐẦU BAY")
                self.tabs.btn_start_fly.setStyleSheet("background:#006600; color:#FFF;")

    @Slot()
    def open_video_file(self):
        if not self.is_vision_mode:
            return
        file_path, _ = QFileDialog.getOpenFileName(self, "Video", "", "Video (*.mp4 *.avi)")
        if file_path:
            # đổi nguồn -> nếu đang REC thì thread đã xử lý dừng
            self.camera_view.thread.set_source(file_path)

    @Slot()
    def open_live_camera(self):
        if not self.is_vision_mode:
            return
        self.camera_view.thread.set_source(0)

    # ====================================================================
    # NEW: REC TOGGLE
    # ====================================================================
    @Slot()
    def toggle_record_video(self):
        if not self.is_vision_mode:
            self.tabs.txt_status.append("⚠ REC chỉ dùng trong MODE 2 (AUTO FLIGHT).")
            return

        # toggle REC (xử lý trong thread)
        self.camera_view.thread.request_toggle_recording()

    @Slot(bool, str)
    def on_recording_state_changed(self, is_recording: bool, path: str):
        # update UI button
        self.tabs.set_rec_button_state(is_recording)

        # log
        if is_recording:
            self.tabs.txt_status.append(f">>> 🎬 START REC: {path}")
        else:
            if path:
                self.tabs.txt_status.append(f">>> ✅ SAVED VIDEO: {path}")
            else:
                self.tabs.txt_status.append(">>> ⏹ STOP REC")

    # ====================================================================
    #           [CẬP NHẬT] LOGIC ĐIỀU KHIỂN & ĐỒNG BỘ SLIDER
    # ====================================================================
    def manual_control_update(self):
        """Khi kéo thanh trượt Throttle"""
        raw_val = self.tabs.slider_th.value()

        # 1. Nếu CHƯA ARM -> Chặn tuyệt đối
        if not self.armed:
            print(f"🚫 BLOCKED: Kéo ga ({raw_val}) khi chưa ARM!")
            return

        # 2. Nếu ĐÃ ARM -> Ép ga tối thiểu là 1140 (MIN_SPIN_THROTTLE)
        if raw_val < MIN_SPIN_THROTTLE:
            self.current_throttle = MIN_SPIN_THROTTLE
        else:
            self.current_throttle = raw_val

        self.send_raw_uart(f"CTRL:{self.current_throttle}:0:0:0\n")

    def toggle_arm(self):
        """Nút ARM/DISARM với tính năng Đồng bộ thanh trượt"""
        if not self.armed:
            # === ARM ===
            if self.tabs.slider_th.value() > 1150:
                QMessageBox.critical(self, "NGUY HIỂM", "Vui lòng kéo cần ga về thấp nhất trước khi ARM!")
                return

            self.armed = True
            self.is_stabilizing = True
            self.can_fly = False
            self.current_throttle = MIN_SPIN_THROTTLE

            # [SYNC] Tự động đẩy thanh trượt lên mức Idle (1140)
            self.tabs.slider_th.setValue(MIN_SPIN_THROTTLE)

            self.tabs.btn_arm.setText("STOP / DISARM")
            self.tabs.btn_arm.setStyleSheet("background:#900; color:#FFF; font-weight:bold;")
            self.tabs.txt_status.append(f">>> ARMING! IDLE SPIN {MIN_SPIN_THROTTLE}...")

            self.stabilize_timer.start(STABILIZE_TIME_MS)
        else:
            # === DISARM ===
            self.armed = False
            self.is_stabilizing = False
            self.can_fly = False
            self.stabilize_timer.stop()
            self.current_throttle = 1000

            # dừng REC nếu đang ghi (tránh quên)
            if self.camera_view.thread.is_recording():
                self.camera_view.thread.request_toggle_recording()

            # 1. Gửi lệnh tắt máy (1000) ngay lập tức
            self.send_raw_uart(f"CTRL:{IDLE_THROTTLE}:0:0:0\n")

            # 2. [SYNC] Kéo thanh trượt về 0 (Block Signal để không in log lỗi)
            self.tabs.slider_th.blockSignals(True)
            self.tabs.slider_th.setValue(IDLE_THROTTLE)
            self.tabs.slider_th.blockSignals(False)

            self.tabs.btn_arm.setText("1. ARM MOTORS")
            self.tabs.btn_arm.setStyleSheet("background:#222; border:2px solid #555; color:#F55; font-weight:bold;")
            self.tabs.txt_status.append(">>> DISARMED. MOTORS STOP.")

    @Slot(dict)
    def process_vision_control(self, vision_data):
        if not self.is_vision_mode or not self.armed:
            return

        if self.is_stabilizing or not self.can_fly:
            cmd = f"CTRL:{MIN_SPIN_THROTTLE}:0:0:0\n"
            self.send_raw_uart(cmd)
            return

        error_pixel = vision_data.get("steering", 0)
        yaw_cmd = int(error_pixel * 0.2)
        yaw_cmd = max(-40, min(40, yaw_cmd))

        # Đảm bảo ga tự động luôn >= 1140
        safe_auto_throttle = max(self.current_throttle, MIN_SPIN_THROTTLE)

        cmd = f"CTRL:{safe_auto_throttle}:0:0:{yaw_cmd}\n"
        self.send_raw_uart(cmd)

    @Slot(dict)
    def on_uart_data(self, data):
        try:
            if not isinstance(data, dict):
                s = str(data).strip()
                if s.startswith('{') and s.endswith('}'):
                    data = json.loads(s)
                else:
                    return

            roll = data.get("roll", 0)
            pitch = data.get("pitch", 0)
            yaw = data.get("yaw", 0)
            v = data.get("v", 0)

            self.drone_3d.set_angles(roll, pitch, yaw)

            if not self.armed:
                stt = "DISARMED"
                clr = "#555"
            elif self.is_stabilizing:
                stt = "STABILIZING..."
                clr = "#FF0"
            elif not self.can_fly:
                stt = "READY (IDLE SPIN)"
                clr = "#0AA"
            else:
                stt = "FLYING"
                clr = "#0F0"

            self.lbl_telemetry.setText(f"[{stt}] R:{roll:.1f} P:{pitch:.1f} Bat:{v}V")
            self.lbl_telemetry.setStyleSheet(f"font-size:18px; color:{clr}; font-weight:bold;")

            self.tabs.plot_roll.update(roll)
            self.tabs.plot_pitch.update(pitch)
            self.tabs.plot_yaw.update(yaw)

        except Exception:
            pass

    def on_stabilize_finished(self):
        if self.armed:
            self.is_stabilizing = False
            self.tabs.txt_status.append(">>> STABILIZED.")
            if self.is_vision_mode:
                self.tabs.btn_start_fly.setEnabled(True)
                self.tabs.btn_start_fly.setText("2. BẮT ĐẦU BAY")
                self.tabs.btn_start_fly.setStyleSheet("background:#006600; color:#FFF; font-weight:bold;")
            else:
                self.tabs.txt_status.append("⚠ MODE TUNING: Manual Only.")

    def start_mission(self):
        if self.armed and not self.is_stabilizing and self.is_vision_mode:
            self.can_fly = True
            self.tabs.txt_status.append(">>> MISSION STARTED!")
            self.tabs.btn_start_fly.setText("FLYING ACTIVE")
            self.tabs.btn_start_fly.setEnabled(False)

    @Slot(str, float, float, float)
    def send_pid_to_uart(self, axis, kp, ki, kd):
        cmd = f"PID:{axis}:{kp:.3f}:{ki:.3f}:{kd:.3f}\n"
        self.tabs.txt_status.append(f">> Sending PID: {cmd.strip()}")
        self.send_raw_uart(cmd)

    def send_motor_test(self, motor_id, percent):
        if self.armed:
            self.tabs.txt_status.append("⚠ WARNING: Cannot test motor while ARMED!")
            return
        cmd = f"MTEST:{motor_id}:{1000 + percent*10}\n"
        self.send_raw_uart(cmd)

    def send_raw_uart(self, s):
        if not self.ready_to_send:
            return
        try:
            ser_obj = None
            if hasattr(self.uart, 'serial'):
                ser_obj = self.uart.serial
            elif hasattr(self.uart, 'ser'):
                ser_obj = self.uart.ser
            elif hasattr(self.uart, 'port') and hasattr(self.uart.port, 'write'):
                ser_obj = self.uart.port

            if ser_obj and ser_obj.is_open:
                ser_obj.write(s.encode('utf-8'))
                ser_obj.flush()
                time.sleep(0.015)
                if not s.startswith("CTRL"):
                    print(f"[TX] {s.strip()}")
            else:
                if not hasattr(self, '_logged_error'):
                    print(f"❌ ERR: Serial chưa mở! (Lệnh: {s.strip()})")
                    self._logged_error = True
        except Exception as e:
            print(f"❌ EXCEPTION UART: {e}")

    def closeEvent(self, event):
        # dừng rec nếu đang ghi
        if self.camera_view.thread.is_recording():
            self.camera_view.thread.request_toggle_recording()

        self.camera_view.close_camera()
        self.uart.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    apply_modern_style(app)
    window = MainPiGUI()
    window.show()
    sys.exit(app.exec())
