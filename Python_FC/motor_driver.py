import lgpio

# ====== CONFIG ======
PWM_FREQ = 50
MIN_PULSE = 1000
MAX_PULSE = 2000

class MotorController:
    def __init__(self, motor_pins):
        """
        motor_pins = [pin_M1, pin_M2, pin_M3, pin_M4]
        """
        self.pins = motor_pins
        self.h = None # Khởi tạo là None
        
        try:
            self.h = lgpio.gpiochip_open(0)
        except Exception as e:
            print(f"Lỗi mở chip GPIO: {e}")
            raise e

        if self.h < 0:
            raise RuntimeError("Không mở được GPIO chip")

        print(f"Motor Driver Init on pins: {self.pins}")
        for pin in self.pins:
            try:
                lgpio.gpio_claim_output(self.h, pin)
                lgpio.tx_pwm(self.h, pin, PWM_FREQ, 0.0)
            except Exception as e:
                print(f"Lỗi claim pin {pin}: {e}")

    # ----------------------------------------
    def pulse_to_duty(self, pulse):
        # 50Hz = 20ms = 20000us
        return (pulse / 20000.0) * 100.0

    # ----------------------------------------
    def set_motor(self, index, pulse):
        # Nếu handle đã đóng (None) thì không làm gì cả
        if self.h is None: 
            return

        if 0 <= index < len(self.pins):
            pulse = max(MIN_PULSE, min(MAX_PULSE, int(pulse)))
            duty = self.pulse_to_duty(pulse)
            try:
                lgpio.tx_pwm(self.h, self.pins[index], PWM_FREQ, duty)
            except lgpio.error:
                # Nếu lỗi handle trong lúc chạy, coi như đã đóng
                self.h = None

    # ----------------------------------------
    def set_all(self, pulse):
        if self.h is None: return
        duty = self.pulse_to_duty(pulse)
        for pin in self.pins:
            try:
                lgpio.tx_pwm(self.h, pin, PWM_FREQ, duty)
            except:
                pass

    # ----------------------------------------
    def set_all_zero(self):
        if self.h is None: return
        for pin in self.pins:
            try:
                lgpio.tx_pwm(self.h, pin, PWM_FREQ, 0.0)
            except:
                pass

    # ----------------------------------------
    # [QUAN TRỌNG] HÀM STOP ĐÃ SỬA LỖI
    # ----------------------------------------
    def stop_all(self):
        # Nếu đã đóng rồi (None) thì thoát ngay, không báo lỗi nữa
        if self.h is None:
            return

        print("Stopping motors...")
        duty = self.pulse_to_duty(1000)
        
        # 1. Cố gắng gửi lệnh dừng động cơ
        for pin in self.pins:
            try:
                lgpio.tx_pwm(self.h, pin, PWM_FREQ, duty)
            except Exception as e:
                pass # Bỏ qua lỗi nếu không gửi được lệnh dừng

        # 2. Đóng chip GPIO và gán về None
        try:
            lgpio.gpiochip_close(self.h)
        except Exception:
            pass
        
        self.h = None # Đánh dấu là đã đóng
        print("Driver Closed Safely.")