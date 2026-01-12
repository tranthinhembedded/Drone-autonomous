from PySide6.QtWidgets import (QTabWidget, QWidget, QVBoxLayout, QPushButton,
                               QGroupBox, QTextEdit, QHBoxLayout, QSlider, QSpinBox, QLabel, QGridLayout)
from PySide6.QtCore import Qt, Signal
from widgets.motor_test import MotorTestTab
from widgets.pid_plot import PIDPlot
from widgets.pid_box import PIDBox

# =========================================================================
# WIDGET THANH TRƯỢT TINH CHỈNH (Custom Slider)
# =========================================================================
class TuningSlider(QWidget):
    """Widget con gồm: Label - Slider - SpinBox"""
    param_changed = Signal(str, int)

    def __init__(self, name, key, min_val, max_val, init_val):
        super().__init__()
        self.key = key

        l = QHBoxLayout()
        l.setContentsMargins(0, 0, 0, 0)

        lbl = QLabel(name)
        lbl.setFixedWidth(70)  # Cố định chiều rộng nhãn

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(min_val, max_val)
        self.slider.setValue(init_val)

        # Ô nhập số (cho chính xác)
        self.spin = QSpinBox()
        self.spin.setRange(min_val, max_val)
        self.spin.setValue(init_val)
        self.spin.setFixedWidth(60)

        # Đồng bộ Slider và SpinBox
        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)

        # Gửi tín hiệu khi giá trị thay đổi
        self.slider.valueChanged.connect(lambda v: self.param_changed.emit(self.key, v))

        l.addWidget(lbl)
        l.addWidget(self.slider)
        l.addWidget(self.spin)
        self.setLayout(l)


# =========================================================================
# WIDGET TAB CHÍNH
# =========================================================================
class ControlTabs(QTabWidget):
    vision_param_signal = Signal(str, int)
    request_open_file = Signal()
    request_camera = Signal()
    pid_update_signal = Signal(str, float, float, float)  # Signal PID gửi ra Main

    # NEW: signal toggle REC
    request_toggle_record = Signal()

    def __init__(self):
        super().__init__()
        # Style cho Tab Bar to và dễ bấm
        self.setStyleSheet("QTabBar::tab { height: 45px; width: 130px; font-weight: bold; font-size: 14px; }")

        # -----------------------------------------------------------------
        # TAB 1: DASHBOARD
        # -----------------------------------------------------------------
        dashboard_w = QWidget()
        dash_layout = QVBoxLayout()

        # --- A. MANUAL CONTROL (ĐIỀU KHIỂN BAY) ---
        grp_man = QGroupBox("1. ĐIỀU KHIỂN BAY")
        man_layout = QVBoxLayout()

        # 1. Thanh ga (Throttle)
        th_l = QHBoxLayout()
        th_l.addWidget(QLabel("THROTTLE (GA):"))

        self.slider_th = QSlider(Qt.Horizontal)
        self.slider_th.setRange(1000, 2000)  # PWM 1000-2000
        self.slider_th.setValue(1000)

        self.spin_th = QSpinBox()
        self.spin_th.setRange(1000, 2000)
        self.spin_th.setValue(1000)

        self.slider_th.valueChanged.connect(self.spin_th.setValue)
        self.spin_th.valueChanged.connect(self.slider_th.setValue)

        th_l.addWidget(self.slider_th)
        th_l.addWidget(self.spin_th)
        man_layout.addLayout(th_l)

        # 2. Hàng nút lệnh: ARM và START
        btn_layout = QHBoxLayout()

        self.btn_arm = QPushButton("1. ARM MOTORS")
        self.btn_arm.setMinimumHeight(45)
        self.btn_arm.setStyleSheet("background:#222; border:2px solid #555; color:#F55; font-weight:bold; font-size:14px;")

        # Nút Bắt đầu bay (Mặc định Disable)
        self.btn_start_fly = QPushButton("2. BẮT ĐẦU BAY 🚀")
        self.btn_start_fly.setMinimumHeight(45)
        self.btn_start_fly.setEnabled(False)  # Chưa ARM thì chưa bấm được
        self.btn_start_fly.setStyleSheet("""
            QPushButton { background:#111; color:#555; border:1px solid #333; font-weight:bold; }
            QPushButton:enabled { background:#006600; color:#FFF; border:2px solid #0F0; }
        """)

        btn_layout.addWidget(self.btn_arm)
        btn_layout.addWidget(self.btn_start_fly)
        man_layout.addLayout(btn_layout)

        grp_man.setLayout(man_layout)
        dash_layout.addWidget(grp_man)

        # --- B. PID TUNING SECTION ---
        grp_pid = QGroupBox("2. PID TUNING (LIVE)")
        pid_layout = QHBoxLayout()  # Xếp ngang

        # Box 1: Pitch & Roll (Dùng chung)
        self.pid_pr = PIDBox("Pitch & Roll (Chung)")
        self.pid_pr.kp.setValue(1.5); self.pid_pr.ki.setValue(0.0); self.pid_pr.kd.setValue(0.8)
        self.pid_pr.pid_submitted.connect(lambda p, i, d: self.pid_update_signal.emit("PR", p, i, d))

        # Box 2: Yaw (Riêng)
        self.pid_yaw = PIDBox("Yaw (Riêng)")
        self.pid_yaw.kp.setValue(3.0); self.pid_yaw.ki.setValue(0.0); self.pid_yaw.kd.setValue(0.0)
        self.pid_yaw.pid_submitted.connect(lambda p, i, d: self.pid_update_signal.emit("YAW", p, i, d))

        pid_layout.addWidget(self.pid_pr)
        pid_layout.addWidget(self.pid_yaw)
        grp_pid.setLayout(pid_layout)
        dash_layout.addWidget(grp_pid)

        # --- C. VISION SOURCE & TUNING ---
        vision_grp = QGroupBox("3. VISION XỬ LÝ ẢNH")
        vision_layout = QVBoxLayout()

        # 1. Các nút chọn nguồn Video + REC
        src_layout = QHBoxLayout()

        self.btn_file = QPushButton("📂 LOAD VIDEO FILE")
        self.btn_file.setStyleSheet("background:#004444; color:#0FF; font-weight:bold; padding:6px;")
        self.btn_file.clicked.connect(self.request_open_file.emit)

        self.btn_cam = QPushButton("🎥 LIVE CAMERA")
        self.btn_cam.setStyleSheet("background:#440044; color:#F0F; font-weight:bold; padding:6px;")
        self.btn_cam.clicked.connect(self.request_camera.emit)

        # NEW: REC button (1 nút toggle)
        self.btn_rec_video = QPushButton("⚫ REC VIDEO")
        self.btn_rec_video.setStyleSheet("""
            QPushButton {
                background:#333;
                color:#DDD;
                font-weight:bold;
                padding:6px;
                border:2px solid #555;
            }
        """)
        self.btn_rec_video.clicked.connect(self.request_toggle_record.emit)

        src_layout.addWidget(self.btn_file)
        src_layout.addWidget(self.btn_cam)
        src_layout.addWidget(self.btn_rec_video)
        vision_layout.addLayout(src_layout)

        # 2. Danh sách các Slider tinh chỉnh (FULL LIST)
        vision_grid = QGridLayout()
        sliders = [
            ("L-Min", "l_min", 0, 255, 200),        # Ngưỡng lọc sáng
            ("W-Top", "w_top", 10, 160, 80),        # Độ rộng trên ROI
            ("W-Bot", "w_bot", 10, 160, 140),       # Độ rộng dưới ROI
            ("H-Sky", "h_sky", 0, 120, 80),         # Chiều cao chân trời
            ("Center-X", "center_x", 0, 320, 160),  # Tâm ngang
            ("Offset-Y", "offset_y", 0, 200, 71),   # Dịch chuyển dọc
            ("Align", "align", 0, 320, 160)         # Vị trí drone mong muốn
        ]

        # Tạo lưới slider (2 cột)
        row, col = 0, 0
        for name, key, mn, mx, val in sliders:
            s = TuningSlider(name, key, mn, mx, val)
            s.param_changed.connect(lambda k, v: self.vision_param_signal.emit(k, v))

            vision_grid.addWidget(s, row, col)
            col += 1
            if col > 1:
                col = 0
                row += 1

        vision_layout.addLayout(vision_grid)
        vision_grp.setLayout(vision_layout)
        dash_layout.addWidget(vision_grp)

        # --- D. SYSTEM LOG (Nhật ký) ---
        self.txt_status = QTextEdit()
        self.txt_status.setReadOnly(True)
        self.txt_status.setMaximumHeight(100)
        self.txt_status.setStyleSheet("background:#000; color:#0F0; border:1px solid #333; font-family:Monospace;")
        dash_layout.addWidget(self.txt_status)

        dashboard_w.setLayout(dash_layout)
        self.addTab(dashboard_w, "Dashboard")

        # -----------------------------------------------------------------
        # TAB 2: GRAPHS (Đồ thị cảm biến)
        # -----------------------------------------------------------------
        plot_w = QWidget()
        pl_l = QVBoxLayout()

        self.plot_roll = PIDPlot("ROLL ANGLE")
        self.plot_pitch = PIDPlot("PITCH ANGLE")
        self.plot_yaw = PIDPlot("YAW ANGLE")

        pl_l.addWidget(self.plot_roll)
        pl_l.addWidget(self.plot_pitch)
        pl_l.addWidget(self.plot_yaw)

        plot_w.setLayout(pl_l)
        self.addTab(plot_w, "Graphs")

        # -----------------------------------------------------------------
        # TAB 3: MOTOR TEST (Kiểm tra động cơ)
        # -----------------------------------------------------------------
        self.motor_tab = MotorTestTab()
        self.addTab(self.motor_tab, "Motor Test")

    # NEW: helper để Main đổi màu nút REC
    def set_rec_button_state(self, is_recording: bool):
        if is_recording:
            self.btn_rec_video.setText("⏹ STOP REC")
            self.btn_rec_video.setStyleSheet("""
                QPushButton {
                    background:#AA0000;
                    color:#FFF;
                    font-weight:bold;
                    padding:6px;
                    border:2px solid #FF4444;
                }
            """)
        else:
            self.btn_rec_video.setText("⚫ REC VIDEO")
            self.btn_rec_video.setStyleSheet("""
                QPushButton {
                    background:#333;
                    color:#DDD;
                    font-weight:bold;
                    padding:6px;
                    border:2px solid #555;
                }
            """)
