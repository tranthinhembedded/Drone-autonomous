# widgets/pid_box.py
from PySide6.QtWidgets import (QGroupBox, QGridLayout, QDoubleSpinBox, 
                               QLabel, QPushButton, QSlider, QWidget)
from PySide6.QtCore import Signal, Qt

class PIDBox(QGroupBox):
    # Signal gửi ra: (Kp, Ki, Kd)
    pid_submitted = Signal(float, float, float)

    def __init__(self, title):
        super().__init__(title)
        
        # Style cho GroupBox
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #444;
                border-radius: 6px;
                margin-top: 6px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
                color: #DDD;
            }
        """)

        grid = QGridLayout()
        grid.setVerticalSpacing(8)
        grid.setHorizontalSpacing(10)

        # --- CẤU HÌNH GIỚI HẠN (RANGE) HỢP LÝ CHO DRONE ---
        # Kp: 0 - 20.0 (Thường dùng khoảng 3.0 - 12.0)
        self.kp, self.kp_slider = self.create_sync_pair(0.0, 20.0, 0.001)
        
        # Ki: 0 - 2.0 (Ki thường rất nhỏ, ví dụ 0.01 - 0.5, để max=2 cho thanh trượt mịn)
        self.ki, self.ki_slider = self.create_sync_pair(0.0, 2.0, 0.001)
        
        # Kd: 0 - 10.0 (Thường dùng khoảng 0.5 - 5.0)
        self.kd, self.kd_slider = self.create_sync_pair(0.0, 10.0, 0.001)

        # --- Layout Grid: Label | Slider | SpinBox ---
        # Hàng 0: Kp
        grid.addWidget(QLabel("Kp"), 0, 0)
        grid.addWidget(self.kp_slider, 0, 1)
        grid.addWidget(self.kp, 0, 2)
        
        # Hàng 1: Ki
        grid.addWidget(QLabel("Ki"), 1, 0)
        grid.addWidget(self.ki_slider, 1, 1)
        grid.addWidget(self.ki, 1, 2)
        
        # Hàng 2: Kd
        grid.addWidget(QLabel("Kd"), 2, 0)
        grid.addWidget(self.kd_slider, 2, 1)
        grid.addWidget(self.kd, 2, 2)

        # --- Nút Gửi ---
        self.btn = QPushButton("GỬI PID")
        self.btn.setCursor(Qt.PointingHandCursor)
        self.btn.setStyleSheet("""
            QPushButton {
                background-color: #006699; 
                color: white; 
                font-weight: bold; 
                padding: 8px; 
                border-radius: 4px;
                margin-top: 5px;
            }
            QPushButton:hover { background-color: #0088CC; }
            QPushButton:pressed { background-color: #004466; }
        """)
        self.btn.clicked.connect(self.emit_values)
        
        # Span nút bấm qua 3 cột
        grid.addWidget(self.btn, 3, 0, 1, 3)

        self.setLayout(grid)

    def create_sync_pair(self, min_val, max_val, step):
        """
        Tạo cặp SpinBox và Slider đồng bộ với nhau.
        Slider sẽ hoạt động trên miền số nguyên (value * 1000).
        """
        # 1. SpinBox (Hiển thị số thực)
        spin = QDoubleSpinBox()
        spin.setDecimals(3)       # 3 chữ số thập phân
        spin.setRange(min_val, max_val)
        spin.setSingleStep(step)  # Bước nhảy khi bấm mũi tên
        spin.setFixedWidth(70)    # Cố định chiều rộng cho gọn
        spin.setStyleSheet("background: #222; color: #0FF; border: 1px solid #555;")

        # 2. Slider (Hoạt động với int)
        # Hệ số nhân 1000 để xử lý 3 số lẻ (0.001)
        FACTOR = 1000
        
        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(min_val * FACTOR), int(max_val * FACTOR))
        slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 4px;
                background: #444;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #0088bb;
                width: 14px;
                height: 14px;
                margin: -5px 0; 
                border-radius: 7px;
            }
            QSlider::handle:horizontal:hover {
                background: #00aaff;
            }
        """)

        # 3. Logic Đồng bộ (Sync)
        # Slider thay đổi -> Cập nhật SpinBox (chia 1000)
        slider.valueChanged.connect(lambda val: spin.setValue(val / FACTOR))

        # SpinBox thay đổi -> Cập nhật Slider (nhân 1000)
        # Lưu ý: dùng round để tránh sai số float (ví dụ 1.00000001)
        spin.valueChanged.connect(lambda val: slider.setValue(int(round(val * FACTOR))))

        return spin, slider

    def emit_values(self):
        """Gửi tín hiệu ra ngoài khi nhấn nút"""
        self.pid_submitted.emit(self.kp.value(), self.ki.value(), self.kd.value())