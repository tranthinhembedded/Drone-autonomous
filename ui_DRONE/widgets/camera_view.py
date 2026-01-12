import sys
import os
import cv2
import time
import threading
import numpy as np
from collections import deque
from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PySide6.QtCore import Qt, QThread, Signal, Slot
from PySide6.QtGui import QImage, QPixmap

# =========================================================================
# 1. CẤU HÌNH & HẰNG SỐ
# =========================================================================
WIDTH  = 320
HEIGHT = 240
LANE_WIDTH_PX = 200
MIN_PIXELS = 50
BORDER = "#555"

# =========================================================================
# 2. CLASS LANE TRACKER
# =========================================================================
class LaneTracker:
    def __init__(self, buffer_size=5):
        self.left_fit_history = deque(maxlen=buffer_size)
        self.right_fit_history = deque(maxlen=buffer_size)
        self.avg_left_fit = None
        self.avg_right_fit = None

    def update_lanes(self, left_fit, right_fit):
        if left_fit is not None:
            self.left_fit_history.append(left_fit)
        if right_fit is not None:
            self.right_fit_history.append(right_fit)

        if len(self.left_fit_history) > 0:
            self.avg_left_fit = np.mean(self.left_fit_history, axis=0)
        if len(self.right_fit_history) > 0:
            self.avg_right_fit = np.mean(self.right_fit_history, axis=0)

        return self.avg_left_fit, self.avg_right_fit

# =========================================================================
# 3. LUỒNG XỬ LÝ VIDEO (VIDEO THREAD)
# =========================================================================
class VideoThread(QThread):
    update_images_signal = Signal(QImage, QImage, QImage, QImage)
    control_signal = Signal(dict)
    recording_state_signal = Signal(bool, str)

    def __init__(self):
        super().__init__()
        self.running = True
        self.source = 0
        self.tracker = LaneTracker(buffer_size=5)

        # FPS calculation
        self.prev_time = 0
        self.fps = 0
        self._fps_ema = 0.0

        # Tuning Params
        self.l_min = 180
        self.w_top = 100
        self.w_bot = 140
        self.h_sky = 80
        self.center_x_raw = 133
        self.offset_y_raw = 71
        self.car_align_pos = 160 # Vị trí tâm xe (Align Line)

        # Control Weights
        self.k_vector = 0.7 
        self.k_error = 0.3  

        # Recording
        self._rec_lock = threading.Lock()
        self._toggle_record_requested = False
        self._is_recording = False
        self._writer = None
        self._record_path = ""
        self._record_dir = os.path.join(os.getcwd(), "recordings")
        os.makedirs(self._record_dir, exist_ok=True)

    def request_toggle_recording(self):
        with self._rec_lock:
            self._toggle_record_requested = True

    def is_recording(self) -> bool:
        with self._rec_lock:
            return self._is_recording

    def run(self):
        cap = cv2.VideoCapture(self.source)
        self.prev_time = time.time()

        while self.running:
            ret, frame = cap.read()
            if not ret:
                if isinstance(self.source, str):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                else:
                    self.msleep(100)
                    continue

            frame = cv2.resize(frame, (WIDTH, HEIGHT))

            # --- FPS Calculation ---
            curr_time = time.time()
            dt = curr_time - self.prev_time
            if dt > 0:
                inst_fps = 1.0 / dt
                if self._fps_ema <= 0:
                    self._fps_ema = inst_fps
                else:
                    self._fps_ema = 0.9 * self._fps_ema + 0.1 * inst_fps
                self.fps = int(self._fps_ema)
            self.prev_time = curr_time

            # --- Recording Logic ---
            do_toggle = False
            with self._rec_lock:
                if self._toggle_record_requested:
                    do_toggle = True
                    self._toggle_record_requested = False

            if do_toggle:
                if not self._is_recording:
                    # START
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    self._record_path = os.path.join(self._record_dir, f"rec_{ts}.mp4")
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    rec_fps = self._fps_ema if self._fps_ema > 1 else 20.0
                    rec_fps = max(5.0, min(60.0, float(rec_fps)))
                    self._writer = cv2.VideoWriter(self._record_path, fourcc, rec_fps, (WIDTH, HEIGHT))
                    if self._writer.isOpened():
                        with self._rec_lock: self._is_recording = True
                        self.recording_state_signal.emit(True, self._record_path)
                    else:
                        self.recording_state_signal.emit(False, "")
                else:
                    # STOP
                    saved = self._record_path
                    if self._writer: self._writer.release()
                    self._writer = None
                    with self._rec_lock: self._is_recording = False
                    self.recording_state_signal.emit(False, saved)

            # --- MAIN PROCESSING ---
            processed_frame, binary_mask, sliding_viz, steering_cmd = self.process_lane(frame)
            self.control_signal.emit({"steering": steering_cmd})

            # Write Frame
            if self.is_recording() and self._writer:
                self._writer.write(frame)

            # Convert to QT
            qt_main = self.cv_to_qt(processed_frame, 640, 480)
            qt_orig = self.cv_to_qt(frame, 320, 240)
            bin_rgb = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2RGB)
            qt_bin = self.cv_to_qt(bin_rgb, 320, 240)
            qt_sliding = self.cv_to_qt(sliding_viz, 320, 240)

            self.update_images_signal.emit(qt_main, qt_orig, qt_bin, qt_sliding)
            self.msleep(30)

        if cap: cap.release()
        if self._writer: self._writer.release()

    def cv_to_qt(self, cv_img, w_target, h_target):
        if len(cv_img.shape) == 3:
            rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        else:
            rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        return qt_img.scaled(w_target, h_target, Qt.KeepAspectRatio)

    def set_source(self, new_source):
        if self.is_recording():
            self.request_toggle_recording()
        self.running = False
        self.wait()
        self.source = new_source
        self.running = True
        self.start()

    def process_lane(self, current_frame):
        # 1. Perspective Transform (ROI)
        shift_x = self.center_x_raw - (WIDTH // 2)
        offset_y = self.offset_y_raw - 100
        center_point = (WIDTH // 2) + shift_x
        top_y = self.h_sky + offset_y
        bot_y = HEIGHT + offset_y
        
        top_y = max(0, min(HEIGHT-1, top_y))
        bot_y = max(0, min(HEIGHT, bot_y))

        src_points = np.float32([
            [center_point - self.w_top, top_y], [center_point + self.w_top, top_y],
            [center_point + self.w_bot, bot_y], [center_point - self.w_bot, bot_y]
        ])

        debug_roi = current_frame.copy()
        cv2.polylines(debug_roi, [np.int32(src_points)], True, (0, 0, 255), 2)

        hls = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls, np.array([0, self.l_min, 0]), np.array([180, 255, 255]))
        warped_mask, inv_matrix = self.warp_image(mask, src_points)

        left_fit_new, right_fit_new, sliding_viz = self.fit_polynomial(warped_mask)
        left_fit, right_fit = self.tracker.update_lanes(left_fit_new, right_fit_new)

        # 2. DRAW & CALCULATE LOGIC
        warp_zero = np.zeros_like(warped_mask).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        ploty = np.linspace(0, HEIGHT - 1, HEIGHT)

        steering_final = 0
        vector_val = 0
        pos_error = 0
        
        # Mặc định vẽ vạch căn xe
        cv2.line(color_warp, (self.car_align_pos, 0), (self.car_align_pos, HEIGHT), (255, 255, 255), 1)

        if left_fit is not None and right_fit is not None:
            try:
                left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
                right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

                # Tô màu vùng làn đường (Màu xanh lá)
                pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
                pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
                pts = np.hstack((pts_left, pts_right))
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

                # --- TÍNH TOÁN & VẼ ĐƯỜNG CONG TÂM (Theo yêu cầu: Nối các tâm) ---
                path_points = []
                step = 20
                for i in range(0, len(ploty), step):
                    y_pt = int(ploty[i])
                    lx = left_fit[0] * y_pt ** 2 + left_fit[1] * y_pt + left_fit[2]
                    rx = right_fit[0] * y_pt ** 2 + right_fit[1] * y_pt + right_fit[2]
                    cx = int((lx + rx) / 2)
                    path_points.append((cx, y_pt))

                    # [VISUAL CŨ] Vẽ chấm đỏ tại các điểm tâm để dễ nhìn độ cong
                    cv2.circle(color_warp, (cx, y_pt), 4, (0, 0, 255), -1)

                if len(path_points) > 1:
                    # [VISUAL CŨ] Vẽ đường nối màu vàng (Yellow Curve)
                    cv2.polylines(color_warp, [np.array(path_points)], False, (0, 255, 255), 3)
                    
                    # --- LOGIC ĐIỀU KHIỂN (Giữ nguyên thuật toán Vector) ---
                    top_pt = path_points[0]   # Xa nhất
                    bot_pt = path_points[-1]  # Gần nhất

                    # 1. Vector (Độ cong): Top_X - Bot_X
                    vector_val = top_pt[0] - bot_pt[0]

                    # 2. Error (Độ lệch): Bot_X - Align_Line
                    pos_error = bot_pt[0] - self.car_align_pos

                    # 3. Tổng hợp PID (Đảo dấu -1 để lái đúng hướng)
                    raw_steering = (pos_error * self.k_error) + (vector_val * self.k_vector)
                    steering_final = -1 * raw_steering

                    # Vẽ đường Vector mờ (để debug) thay vì đường hồng to
                    cv2.line(color_warp, bot_pt, top_pt, (255, 0, 255), 1)

            except Exception as e:
                print(f"Lane Calc Error: {e}")

        # Unwarp & Merge
        newwarp = cv2.warpPerspective(color_warp, inv_matrix, (WIDTH, HEIGHT))
        result = cv2.addWeighted(debug_roi, 1, newwarp, 0.5, 0)

        # --- HIỂN THỊ TEXT THÔNG SỐ ---
        direction_msg = "STRAIGHT"
        msg_color = (0, 255, 0)
        thresh_turn = 5

        if steering_final < -thresh_turn:
            direction_msg = "TURN RIGHT >>"
            msg_color = (0, 0, 255)
        elif steering_final > thresh_turn:
            direction_msg = "<< TURN LEFT"
            msg_color = (0, 0, 255)

        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(direction_msg, font, 0.8, 2)[0]
        text_x = (WIDTH - text_size[0]) // 2
        
        cv2.putText(result, direction_msg, (text_x, HEIGHT - 40), font, 0.8, (0, 0, 0), 5)
        cv2.putText(result, direction_msg, (text_x, HEIGHT - 40), font, 0.8, msg_color, 2)

        cv2.putText(result, f"VEC: {vector_val:.1f}", (10, 30), font, 0.6, (0, 255, 255), 2)
        cv2.putText(result, f"ERR: {pos_error:.1f}", (10, 55), font, 0.6, (0, 255, 255), 2)
        cv2.putText(result, f"OUT: {steering_final:.1f}", (10, 80), font, 0.6, (0, 255, 0), 2)

        # Thanh Bar hiển thị lực lái
        bar_len = int(steering_final * 2)
        center_w = WIDTH // 2
        cv2.rectangle(
            result,
            (center_w, HEIGHT - 20),
            (center_w + bar_len, HEIGHT - 10),
            (255, 0, 255), 
            -1
        )

        return result, mask, sliding_viz, int(steering_final)

    def warp_image(self, img, points_src):
        points_dst = np.float32([[50, 0], [WIDTH - 50, 0], [WIDTH - 50, HEIGHT], [50, HEIGHT]])
        matrix = cv2.getPerspectiveTransform(points_src, points_dst)
        warped = cv2.warpPerspective(img, matrix, (WIDTH, HEIGHT))
        inv_matrix = cv2.getPerspectiveTransform(points_dst, points_src)
        return warped, inv_matrix

    def fit_polynomial(self, binary_warped):
        leftx, lefty, rightx, righty, out_img = self.find_lane_pixels(binary_warped)
        left_fit = None
        right_fit = None
        found_left = len(leftx) > MIN_PIXELS
        found_right = len(rightx) > MIN_PIXELS

        if found_left and found_right:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
        elif not found_left and found_right:
            right_fit = np.polyfit(righty, rightx, 2)
            left_fit = np.copy(right_fit)
            left_fit[2] -= LANE_WIDTH_PX
        elif found_left and not found_right:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.copy(left_fit)
            right_fit[2] += LANE_WIDTH_PX
        return left_fit, right_fit, out_img
   def calculate_uart_packet(self, steering_yaw, pitch_cmd, roll_cmd, throttle_current=1500):
        # Giới hạn giá trị an toàn
        safe_yaw = max(-40, min(40, int(steering_yaw)))
        safe_pitch = max(-40, min(0, int(pitch_cmd)))
        safe_roll = int(roll_cmd)
        
        # Format: CTRL:{throttle}:{pitch}:{roll}:{yaw}
        packet = f"CTRL:{throttle_current}:{safe_pitch}:{safe_roll}:{safe_yaw}\n"     
        return packet

    def find_lane_pixels(self, binary_warped):
        histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
        midpoint = int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = int(binary_warped.shape[0] // nwindows)
        margin = 30
        minpix = MIN_PIXELS
        leftx_current = leftx_base
        rightx_current = rightx_base

        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_lane_inds = []
        right_lane_inds = []
        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 1)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 1)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]
        return leftx, lefty, rightx, righty, out_img

    def stop(self):
        if self.is_recording():
            self.request_toggle_recording()
        self.running = False
        self.wait()

    @Slot(str, int)
    def update_params(self, key, value):
        if key == 'l_min': self.l_min = value
        elif key == 'w_top': self.w_top = value
        elif key == 'w_bot': self.w_bot = value
        elif key == 'h_sky': self.h_sky = value
        elif key == 'center_x': self.center_x_raw = value
        elif key == 'offset_y': self.offset_y_raw = value
        elif key == 'align': self.car_align_pos = value

# =========================================================================
# 4. WIDGET GIAO DIỆN CHÍNH
# =========================================================================
class CameraView(QGroupBox):
    def __init__(self):
        super().__init__("VISION PROCESSING (SPLIT VIEW)")
        self.setMinimumHeight(600)

        layout = QHBoxLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        sub_style = f"background-color: #111; border: 1px solid {BORDER}; border-radius: 4px;"

        def create_view(title):
            frame = QFrame()
            frame.setStyleSheet(sub_style)
            l = QVBoxLayout(frame)
            l.setContentsMargins(0, 0, 0, 0)
            l.setSpacing(0)
            lbl_t = QLabel(title)
            lbl_t.setAlignment(Qt.AlignCenter)
            lbl_t.setStyleSheet("color: #AAA; font-size: 11px; font-weight: bold; background: #222; padding: 3px;")
            lbl_img = QLabel()
            lbl_img.setAlignment(Qt.AlignCenter)
            lbl_img.setStyleSheet("border: none; background: #000;")
            lbl_img.setScaledContents(True)
            l.addWidget(lbl_t)
            l.addWidget(lbl_img)
            return frame, lbl_img

        self.frame_main, self.lbl_main = create_view("FINAL RESULT (VECTOR + ERROR)")
        layout.addWidget(self.frame_main, stretch=2)

        right_layout = QVBoxLayout()
        right_layout.setSpacing(5)
        self.frame_orig, self.lbl_orig = create_view("1. ORIGINAL / ROI")
        right_layout.addWidget(self.frame_orig)
        self.frame_be, self.lbl_be = create_view("2. SLIDING WINDOWS")
        right_layout.addWidget(self.frame_be)
        self.frame_bin, self.lbl_bin = create_view("3. BINARY MASK")
        right_layout.addWidget(self.frame_bin)
        layout.addLayout(right_layout, stretch=1)

        self.setLayout(layout)
        self.thread = VideoThread()
        self.thread.update_images_signal.connect(self.update_images)
        self.thread.start()

    @Slot(QImage, QImage, QImage, QImage)
    def update_images(self, img_main, img_orig, img_bin, img_sliding):
        self.lbl_main.setPixmap(QPixmap.fromImage(img_main))
        self.lbl_orig.setPixmap(QPixmap.fromImage(img_orig))
        self.lbl_bin.setPixmap(QPixmap.fromImage(img_bin))
        self.lbl_be.setPixmap(QPixmap.fromImage(img_sliding))

    def close_camera(self):
        self.thread.stop()