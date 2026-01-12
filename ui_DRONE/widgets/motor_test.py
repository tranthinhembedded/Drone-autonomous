# widgets/motor_test.py
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QSlider, 
                               QLabel, QPushButton, QGroupBox, QSpinBox)
from PySide6.QtCore import Qt, Signal
from utils.colors import ACCENT, BORDER, PANEL_BG, ACCENT_HOVER

class SingleMotor(QWidget):
    # Signal: (motor_id, value)
    value_changed = Signal(int, int)

    def __init__(self, motor_id, name):
        super().__init__()
        self.motor_id = motor_id
        
        layout = QVBoxLayout()
        
        # Label
        self.lbl_name = QLabel(f"{name} (M{motor_id})")
        self.lbl_name.setAlignment(Qt.AlignCenter)
        self.lbl_name.setStyleSheet("font-weight:bold; color: #A9A9B3;")
        
        # Slider (0 - 100%)
        self.slider = QSlider(Qt.Vertical)
        self.slider.setRange(0, 100)
        self.slider.setStyleSheet(f"""
            QSlider::groove:vertical {{
                background: {BORDER};
                width: 6px;
                border-radius: 3px;
            }}
            QSlider::handle:vertical {{
                background: {ACCENT};
                height: 16px;
                margin: 0 -5px;
                border-radius: 8px;
            }}
        """)
        
        # SpinBox (Hiển thị số)
        self.spin = QSpinBox()
        self.spin.setRange(0, 100)
        self.spin.setStyleSheet(f"background: {PANEL_BG}; color: {ACCENT}; border: 1px solid {BORDER};")
        self.spin.setAlignment(Qt.AlignCenter)

        # Sync Slider & Spinbox
        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)
        
        # Emit Signal when changed
        self.slider.valueChanged.connect(self.emit_change)

        layout.addWidget(self.lbl_name)
        layout.addWidget(self.slider, alignment=Qt.AlignCenter)
        layout.addWidget(self.spin)
        self.setLayout(layout)

    def emit_change(self):
        val = self.slider.value()
        self.value_changed.emit(self.motor_id, val)

    def reset(self):
        self.slider.setValue(0)


class MotorTestTab(QWidget):
    # Signal tổng: gửi (motor_id, throttle_percent) ra ngoài
    send_motor_command = Signal(int, int)

    def __init__(self):
        super().__init__()
        
        main_layout = QVBoxLayout()
        
        # --- 1. WARNING BOX ---
        warn_box = QGroupBox("⚠️ AN TOÀN")
        warn_layout = QVBoxLayout()
        warn_lbl = QLabel("Vui lòng tháo cánh quạt trước khi test động cơ!\nGiá trị gửi đi từ 0% đến 100%.")
        warn_lbl.setStyleSheet("color: #FF5555; font-style: italic;")
        warn_layout.addWidget(warn_lbl)
        warn_box.setLayout(warn_layout)
        warn_box.setStyleSheet(f"border: 1px solid #FF5555; border-radius: 6px;")
        
        main_layout.addWidget(warn_box)

        # --- 2. MOTOR SLIDERS ---
        sliders_layout = QHBoxLayout()
        self.motors = []
        
        # Định nghĩa 4 motor (Quad X config)
        # M1: Front Right, M2: Back Right, M3: Back Left, M4: Front Left (Ví dụ)
        configs = [
            (1, "Front Right"),
            (2, "Back Right"),
            (3, "Back Left"),
            (4, "Front Left")
        ]

        for mid, name in configs:
            m_widget = SingleMotor(mid, name)
            m_widget.value_changed.connect(self.on_motor_change)
            sliders_layout.addWidget(m_widget)
            self.motors.append(m_widget)

        main_layout.addLayout(sliders_layout)

        # --- 3. MASTER & STOP ---
        ctrl_layout = QHBoxLayout()
        
        self.btn_stop = QPushButton("🛑 STOP ALL MOTORS")
        self.btn_stop.setMinimumHeight(50)
        self.btn_stop.setStyleSheet("""
            QPushButton { background-color: #FF4444; color: white; font-weight: bold; font-size: 16px; border-radius: 8px; }
            QPushButton:hover { background-color: #FF2222; }
        """)
        self.btn_stop.clicked.connect(self.stop_all)
        
        ctrl_layout.addWidget(self.btn_stop)
        main_layout.addLayout(ctrl_layout)
        
        self.setLayout(main_layout)

    def on_motor_change(self, mid, val):
        self.send_motor_command.emit(mid, val)

    def stop_all(self):
        for m in self.motors:
            m.reset() # Sẽ trigger signal valueChanged -> gửi 0 về servern