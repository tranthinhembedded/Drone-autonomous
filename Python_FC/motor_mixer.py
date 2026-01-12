# motor_mixer.py

class QuadXMixer:
    def __init__(self, min_cmd=1100, max_cmd=1900):
        """
        CẤU HÌNH ĐỘNG CƠ (QuadX):
        ---------------------------------------------------------
        M1: Trước Phải  (Front Right)
        M2: Sau  Phải   (Back/Rear Right)
        M3: Sau  Trái   (Back/Rear Left)
        M4: Trước Trái  (Front Left)
        ---------------------------------------------------------

        QUY ƯỚC DẤU:
        - r_out > 0: roll dương = nghiêng phải  -> tăng bên TRÁI (M3,M4), giảm bên PHẢI (M1,M2)
        - p_out > 0: pitch dương = chúi xuống   -> tăng phía SAU (M2,M3), giảm phía TRƯỚC (M1,M4)
        """
        self.min_cmd = min_cmd
        self.max_cmd = max_cmd

    def compute(self, throttle, r_out, p_out, y_out=None):
        """
        Tính PWM. y_out hiện chưa dùng (bỏ yaw).
        """

        # M1: Front Right
        m1 = throttle - r_out - p_out

        # M2: Rear Right
        m2 = throttle - r_out + p_out

        # M3: Rear Left
        m3 = throttle + r_out + p_out

        # M4: Front Left
        m4 = throttle + r_out - p_out

        return [self.clamp(m1), self.clamp(m2), self.clamp(m3), self.clamp(m4)]

    def clamp(self, value):
        return max(self.min_cmd, min(self.max_cmd, int(value)))


# --- Compatibility wrapper for ws_server.py ---
_default_mixer = QuadXMixer(min_cmd=1000, max_cmd=2000)

def mix_quad_x(throttle, roll, pitch, yaw=0.0, min_pwm=1000, max_pwm=2000):
    global _default_mixer
    # cập nhật clamp theo ws_server truyền vào
    _default_mixer.min_cmd = min_pwm
    _default_mixer.max_cmd = max_pwm
    m1, m2, m3, m4 = _default_mixer.compute(throttle, roll, pitch, yaw)
    return m1, m2, m3, m4
