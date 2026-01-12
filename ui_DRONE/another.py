import cv2
import numpy as np
import time
import os
from collections import deque

# =========================================================================
# 1. CẤU HÌNH & HẰNG SỐ
# =========================================================================
WIDTH  = 320
HEIGHT = 240
LANE_WIDTH_PX = 200
MIN_PIXELS = 50

# ĐƯỜNG DẪN FILE INPUT
INPUT_VIDEO = "LANEFINAL.mp4" 

# ĐƯỜNG DẪN 3 FILE OUTPUT
OUTPUT_FINAL = "output_1_final.mp4"   # Kết quả cuối cùng kèm thông số
OUTPUT_BINARY = "output_2_binary.mp4" # Ảnh nhị phân (trắng đen)
OUTPUT_BEV = "output_3_bev.mp4"       # Ảnh góc nhìn trên cao (Birds-eye View)

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
# 3. CLASS XỬ LÝ CHÍNH
# =========================================================================
class LaneProcessor:
    def __init__(self):
        self.tracker = LaneTracker(buffer_size=5)

        # --- TUNING PARAMS (Đã cập nhật) ---
        self.l_min = 164        # [L-Min]
        self.w_top = 125        # [W-Top]
        self.w_bot = 140        # [W-Bot]
        self.h_sky = 80         # [H-Sky]
        self.center_x_raw = 160 # [Center-X]
        self.offset_y_raw = 71  # [Offset-Y]
        self.car_align_pos = 160 # [Align]

        # Control Weights
        self.k_vector = 0.7 
        self.k_error = 0.3  
        
        # Flight Params
        self.forward_pitch = -5.0  
        self.turn_threshold = 10.0 

    def warp_image(self, img, points_src):
        points_dst = np.float32([[50, 0], [WIDTH - 50, 0], [WIDTH - 50, HEIGHT], [50, HEIGHT]])
        matrix = cv2.getPerspectiveTransform(points_src, points_dst)
        warped = cv2.warpPerspective(img, matrix, (WIDTH, HEIGHT))
        inv_matrix = cv2.getPerspectiveTransform(points_dst, points_src)
        return warped, inv_matrix

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
        
        return leftx, lefty, rightx, righty, out_img

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

    def process_frame(self, current_frame):
        # 1. Perspective Transform (ROI)
        current_frame = cv2.resize(current_frame, (WIDTH, HEIGHT))
        
        shift_x = self.center_x_raw - (WIDTH // 2)
        offset_y = self.offset_y_raw - 100
        center_point = (WIDTH // 2) + shift_x
        top_y = max(0, min(HEIGHT-1, self.h_sky + offset_y))
        bot_y = max(0, min(HEIGHT, HEIGHT + offset_y))

        src_points = np.float32([
            [center_point - self.w_top, top_y], [center_point + self.w_top, top_y],
            [center_point + self.w_bot, bot_y], [center_point - self.w_bot, bot_y]
        ])

        debug_roi = current_frame.copy()
        cv2.polylines(debug_roi, [np.int32(src_points)], True, (0, 0, 255), 2)

        hls = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls, np.array([0, self.l_min, 0]), np.array([180, 255, 255]))
        
        # --- ẢNH 2: BINARY MASK (Góc nhìn trên cao) ---
        warped_mask, inv_matrix = self.warp_image(mask, src_points)

        left_fit_new, right_fit_new, sliding_viz = self.fit_polynomial(warped_mask)
        left_fit, right_fit = self.tracker.update_lanes(left_fit_new, right_fit_new)

        # 2. CALCULATION & DRAWING
        warp_zero = np.zeros_like(warped_mask).astype(np.uint8)
        
        # --- ẢNH 3: BIRDS-EYE VIEW PATH ---
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        
        ploty = np.linspace(0, HEIGHT - 1, HEIGHT)

        steering_yaw = 0
        pitch_cmd = 0
        roll_cmd = 0
        vector_val = 0
        pos_error = 0
        
        # Vẽ vạch căn xe
        cv2.line(color_warp, (self.car_align_pos, 0), (self.car_align_pos, HEIGHT), (255, 255, 255), 1)

        if left_fit is not None and right_fit is not None:
            try:
                left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
                right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

                # Tô màu làn đường (Xanh lá)
                pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
                pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
                pts = np.hstack((pts_left, pts_right))
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

                # Tính toán đường cong tâm
                path_points = []
                step = 20
                for i in range(0, len(ploty), step):
                    y_pt = int(ploty[i])
                    lx = left_fit[0] * y_pt ** 2 + left_fit[1] * y_pt + left_fit[2]
                    rx = right_fit[0] * y_pt ** 2 + right_fit[1] * y_pt + right_fit[2]
                    cx = int((lx + rx) / 2)
                    path_points.append((cx, y_pt))
                    # Vẽ chấm đỏ
                    cv2.circle(color_warp, (cx, y_pt), 4, (0, 0, 255), -1)

                if len(path_points) > 1:
                    # Vẽ đường nối màu vàng
                    cv2.polylines(color_warp, [np.array(path_points)], False, (0, 255, 255), 3)
                    
                    top_pt = path_points[0]
                    bot_pt = path_points[-1]

                    # --- YAW CALCULATION ---
                    vector_val = top_pt[0] - bot_pt[0]
                    pos_error = bot_pt[0] - self.car_align_pos
                    raw_steering = (pos_error * self.k_error) + (vector_val * self.k_vector)
                    steering_yaw = -1 * raw_steering

                    # --- PITCH/ROLL LOGIC ---
                    if abs(steering_yaw) > self.turn_threshold:
                        pitch_cmd = 0
                        roll_cmd = 0
                    else:
                        pitch_cmd = self.forward_pitch 
                        roll_cmd = 0

                    # Vẽ vector mờ tím
                    cv2.line(color_warp, bot_pt, top_pt, (255, 0, 255), 1)

            except Exception as e:
                pass

        # Unwarp & Merge
        newwarp = cv2.warpPerspective(color_warp, inv_matrix, (WIDTH, HEIGHT))
        
        # --- ẢNH 1: FINAL RESULT ---
        result = cv2.addWeighted(debug_roi, 1, newwarp, 0.5, 0)

        # --- DRAW TEXT INFO ON FINAL RESULT ---
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Hướng rẽ
        direction_msg = "STRAIGHT"
        msg_color = (0, 255, 0)
        thresh_turn = 5
        if steering_yaw < -thresh_turn:
            direction_msg = "TURN RIGHT >>"
            msg_color = (0, 0, 255)
        elif steering_yaw > thresh_turn:
            direction_msg = "<< TURN LEFT"
            msg_color = (0, 0, 255)

        text_size = cv2.getTextSize(direction_msg, font, 0.8, 2)[0]
        text_x = (WIDTH - text_size[0]) // 2
        cv2.putText(result, direction_msg, (text_x, HEIGHT - 20), font, 0.8, (0, 0, 0), 5)
        cv2.putText(result, direction_msg, (text_x, HEIGHT - 20), font, 0.8, msg_color, 2)

        # Thông số 3 trục
        cv2.putText(result, f"YAW : {int(steering_yaw)}", (10, 30), font, 0.6, (0, 255, 0), 2)
        p_color = (0, 255, 255) if pitch_cmd != 0 else (0, 0, 255)
        cv2.putText(result, f"PIT : {int(pitch_cmd)}", (10, 55), font, 0.6, p_color, 2)
        cv2.putText(result, f"ROL : {int(roll_cmd)}", (10, 80), font, 0.6, (255, 255, 0), 2)

        # Thanh Bar YAW
        bar_len = int(steering_yaw * 2)
        center_w = WIDTH // 2
        cv2.rectangle(result, (center_w, HEIGHT - 10), (center_w + bar_len, HEIGHT - 5), (255, 0, 255), -1)

        # --- TRẢ VỀ 3 ẢNH ---
        # 1. Final Result (Màu)
        # 2. Binary Mask (Trắng đen - góc nhìn trên cao)
        # 3. Birds-eye View Path (Màu - đường dẫn và làn xe)
        return result, warped_mask, color_warp

# =========================================================================
# 4. HÀM MAIN XỬ LÝ VIDEO VÀ XUẤT 3 FILE
# =========================================================================
def main():
    if not os.path.exists(INPUT_VIDEO):
        print(f"❌ Lỗi: Không tìm thấy file '{INPUT_VIDEO}'.")
        return

    cap = cv2.VideoCapture(INPUT_VIDEO)
    orig_fps = cap.get(cv2.CAP_PROP_FPS)
    if orig_fps == 0: orig_fps = 30.0 
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"--> Bắt đầu xử lý: {INPUT_VIDEO} ({total_frames} frames)")

    # --- KHỞI TẠO 3 VIDEO WRITER ---
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_final = cv2.VideoWriter(OUTPUT_FINAL, fourcc, orig_fps, (WIDTH, HEIGHT))
    out_binary = cv2.VideoWriter(OUTPUT_BINARY, fourcc, orig_fps, (WIDTH, HEIGHT))
    out_bev = cv2.VideoWriter(OUTPUT_BEV, fourcc, orig_fps, (WIDTH, HEIGHT))

    processor = LaneProcessor()
    frame_count = 0
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Nhận 3 ảnh từ hàm xử lý
        final_frame, binary_frame, bev_frame = processor.process_frame(frame)
        
        # --- XỬ LÝ ẢNH NHỊ PHÂN TRƯỚC KHI GHI ---
        # Ảnh nhị phân là ảnh xám (1 kênh), cần chuyển sang 3 kênh (RGB) để ghi video
        if len(binary_frame.shape) == 2:
             binary_writable = cv2.cvtColor(binary_frame, cv2.COLOR_GRAY2BGR)
        else:
             binary_writable = binary_frame

        # --- GHI VÀO 3 FILE TƯƠNG ỨNG ---
        out_final.write(final_frame)
        out_binary.write(binary_writable)
        out_bev.write(bev_frame) # bev_frame đã là ảnh màu
        
        frame_count += 1
        if frame_count % 50 == 0:
            percent = (frame_count / total_frames) * 100
            print(f"Processing... {percent:.1f}%")

    # --- GIẢI PHÓNG TÀI NGUYÊN ---
    cap.release()
    out_final.release()
    out_binary.release()
    out_bev.release()
    
    duration = time.time() - start_time
    print("="*40)
    print(f"✅ HOÀN TẤT TRONG {duration:.2f} GIÂY!")
    print(f"--> Video 1 (Final):  {OUTPUT_FINAL}")
    print(f"--> Video 2 (Binary): {OUTPUT_BINARY}")
    print(f"--> Video 3 (BEV):    {OUTPUT_BEV}")
    print("="*40)

if __name__ == "__main__":
    main()