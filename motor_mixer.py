class QuadXMixer:
    def __init__(self, min_cmd=1100, max_cmd=1900):
        """
        Bộ trộn tín hiệu theo chuẩn C++ tham khảo (espPIDDrone.txt).
        
        CẤU HÌNH ĐỘNG CƠ:
        ---------------------------------------------------------
        Vị trí          |  Chiều quay (Spin) |  Ký hiệu logic
        ---------------------------------------------------------
        M1: Trước Phải  |  CCW (Ngược chiều) |  FR_CCW
        M2: Sau Phải    |  CW  (Cùng chiều)  |  BR_CW
        M3: Sau Trái    |  CCW (Ngược chiều) |  BL_CCW
        M4: Trước Trái  |  CW  (Cùng chiều)  |  FL_CW
        ---------------------------------------------------------
        """
        self.min_cmd = min_cmd
        self.max_cmd = max_cmd

    def compute(self, throttle, r_out, p_out, y_out):
        """
        Tính toán xung PWM dựa trên công thức từ file C++ tham khảo.
        """
        
        # Công thức từ dòng 100-104 trong espPIDDrone.txt
        
        # M1: Front Right - CCW
        # InputThrottle - InputRoll - InputPitch - InputYaw
        m1 = throttle - r_out - p_out - y_out

        # M2: Rear Right - CW
        # InputThrottle - InputRoll + InputPitch + InputYaw
        m2 = throttle - r_out + p_out + y_out

        # M3: Rear Left - CCW
        # InputThrottle + InputRoll + InputPitch - InputYaw
        m3 = throttle + r_out + p_out - y_out

        # M4: Front Left - CW
        # InputThrottle + InputRoll - InputPitch + InputYaw
        m4 = throttle + r_out - p_out + y_out

        # Kẹp giá trị trong khoảng an toàn
        return [self.clamp(m1), self.clamp(m2), self.clamp(m3), self.clamp(m4)]

    def clamp(self, value):
        return max(self.min_cmd, min(self.max_cmd, int(value)))