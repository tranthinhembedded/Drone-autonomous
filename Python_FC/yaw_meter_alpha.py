import smbus2
import time
import math
import threading
from kalman_filter import KalmanAngle


class YawMeterAlpha:

    def __init__(self):
        self.bus = smbus2.SMBus(1)

        # HMC5883L address
        self.HMC = 0x1E
        self.init_hmc()

        # MPU6050 address
        self.MPU = 0x68
        self.gyroZ_reg = 0x47

        # Kalman for yaw
        self.kalmanYaw = KalmanAngle()

        # Values
        self.yaw_raw = 0
        self.yaw_kalman = 0

        # Zero heading
        self.yaw_zero = None

        # Start thread
        self.thread = threading.Thread(target=self.measure_yaw_thread)
        self.thread.start()

    # ===============================
    # INIT HMC5883L
    # ===============================
    def init_hmc(self):
        self.bus.write_byte_data(self.HMC, 0x00, 0x70)
        self.bus.write_byte_data(self.HMC, 0x01, 0x20)
        self.bus.write_byte_data(self.HMC, 0x02, 0x00)
        time.sleep(0.1)

    # ===============================
    # READ RAW 16bit
    # ===============================
    def read_raw(self, addr, reg):
        h = self.bus.read_byte_data(addr, reg)
        l = self.bus.read_byte_data(addr, reg + 1)
        v = (h << 8) | l
        if v > 32767:
            v -= 65536
        return v

    # ===============================
    # ABSOLUTE YAW FROM MAGNETOMETER
    # ===============================
    def get_mag_yaw(self):
        X = self.read_raw(self.HMC, 0x03)
        Z = self.read_raw(self.HMC, 0x05)
        Y = self.read_raw(self.HMC, 0x07)

        yaw = math.degrees(math.atan2(Y, X))
        if yaw < 0:
            yaw += 360
        return yaw

    # ===============================
    # GYRO Z RATE (deg/s)
    # ===============================
    def get_gyro_z(self):
        gz = self.read_raw(self.MPU, self.gyroZ_reg)
        return gz / 131.0

    # ===============================
    # MEASUREMENT THREAD
    # ===============================
    def measure_yaw_thread(self):

        # Cho cảm biến ổn định
        time.sleep(1)

        # Lấy yaw từ mag
        first_yaw = self.get_mag_yaw()

        # Khởi tạo Kalman
        self.kalmanYaw.setAngle(first_yaw)
        self.yaw_kalman = first_yaw

        # Ghi mốc zero
        self.yaw_zero = first_yaw

        prev_t = time.time()

        while True:

            now = time.time()
            dt = now - prev_t
            prev_t = now

            yaw_mag = self.get_mag_yaw()     # absolute yaw
            yaw_rate = self.get_gyro_z()     # gyro Z

            # Kalman yaw
            self.yaw_kalman = self.kalmanYaw.getAngle(yaw_mag, yaw_rate, dt)

            # Save raw yaw for debugging
            self.yaw_raw = yaw_mag

            time.sleep(0.01)

    # ===============================
    # PUBLIC API
    # ===============================
    def get_raw_yaw(self):
        return self.yaw_raw

    def get_kalman_yaw(self):
        return self.yaw_kalman

    def get_relative_yaw(self):
        """
        Yaw = yaw_kalman - yaw_zero
        Always normalized to [-180..180]
        Deadzone → if inside ±5°, return 0.
        """
        if self.yaw_zero is None:
            return 0

        diff = self.yaw_kalman - self.yaw_zero

        # normalize to [-180..180]
        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360

        # deadzone
        DEADBAND = 5.0  # ±5°
        if abs(diff) <= DEADBAND:
            return 0

        return diff
