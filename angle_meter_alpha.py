import smbus2
import time
import math
from kalman_filter import KalmanFilter
import threading

class AngleMeterAlpha():

    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846

    GYRO_SCALE = 131.0  # 131 LSB = 1 deg/sec for default ±250 dps
    ACCEL_SCALE = 16384.0  # 16384 LSB = 1g for default ±2g

    bus = smbus2.SMBus(1)
    DeviceAddress = 0x68

    # Kalman filter instances
    kalmanX = KalmanFilter()
    kalmanY = KalmanFilter()

    # Kalman filter angles
    kalAngleX = 0.0
    kalAngleY = 0.0

    # Kalman filter timers
    last_time = time.time()

    # Thread safety variables
    lock = threading.Lock()
    roll = 0.0
    pitch = 0.0

    def __init__(self):
        self.MPU_Init()
        self.kalmanX.setAngle(0.0)
        self.kalmanY.setAngle(0.0)
        self.last_time = time.time()

    def MPU_Init(self):
        # Write to sample rate register
        self.bus.write_byte_data(self.DeviceAddress, 0x19, 7)
        # Write to power management register
        self.bus.write_byte_data(self.DeviceAddress, 0x6B, 1)
        # Write to Configuration register
        self.bus.write_byte_data(self.DeviceAddress, 0x1A, 0)
        # Write to Gyro configuration register
        self.bus.write_byte_data(self.DeviceAddress, 0x1B, 0)
        # Write to interrupt enable register
        self.bus.write_byte_data(self.DeviceAddress, 0x38, 1)

    def read_raw_data(self, addr):
        # Accel and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.DeviceAddress, addr)
        low = self.bus.read_byte_data(self.DeviceAddress, addr + 1)
        value = ((high << 8) | low)
        # to get signed value from mpu6050
        if value > 32768:
            value = value - 65536
        return value

    def read_angles_loop(self):
        # Complementary filter initial state
        compAngleX = 0.0
        compAngleY = 0.0
        gyroXAngle = 0.0
        gyroYAngle = 0.0

        try:
            while True:
                # Read Accelerometer raw values
                accX = self.read_raw_data(0x3B)
                accY = self.read_raw_data(0x3D)
                accZ = self.read_raw_data(0x3F)

                # Read Gyroscope raw values
                gyroX = self.read_raw_data(0x43)
                gyroY = self.read_raw_data(0x45)
                gyroZ = self.read_raw_data(0x47)

                # Convert raw to scaled units
                Ax = accX / self.ACCEL_SCALE
                Ay = accY / self.ACCEL_SCALE
                Az = accZ / self.ACCEL_SCALE

                Gx = gyroX / self.GYRO_SCALE
                Gy = gyroY / self.GYRO_SCALE
                Gz = gyroZ / self.GYRO_SCALE

                # Calculate dt
                curr_time = time.time()
                dt = curr_time - self.last_time
                self.last_time = curr_time
                if dt <= 0:
                    dt = 0.001

                # Calculate roll and pitch from accelerometer (degrees)
                roll_acc = math.atan2(Ay, Az) * self.RAD_TO_DEG
                pitch_acc = math.atan2(-Ax, math.sqrt(Ay * Ay + Az * Az)) * self.RAD_TO_DEG

                # Gyro rates (deg/s)
                gyroXRate = Gx
                gyroYRate = Gy

                # --- FIXED: integrate gyro angles correctly ---
                gyroXAngle += gyroXRate * dt
                gyroYAngle += gyroYRate * dt

                # Complementary filter (example)
                alpha = 0.98
                compAngleX = alpha * (compAngleX + gyroXRate * dt) + (1 - alpha) * roll_acc
                compAngleY = alpha * (compAngleY + gyroYRate * dt) + (1 - alpha) * pitch_acc

                # Kalman filter update
                self.kalAngleX = self.kalmanX.getAngle(roll_acc, gyroXRate, dt)
                self.kalAngleY = self.kalmanY.getAngle(pitch_acc, gyroYRate, dt)

                # Publish to shared state
                with self.lock:
                    self.roll = self.kalAngleX
                    self.pitch = self.kalAngleY

                time.sleep(0.001)
        except Exception as exc:
            print(exc)

    def measure(self):
        t = threading.Thread(target=self.read_angles_loop, daemon=True)
        t.start()

    def get_roll_pitch(self):
        with self.lock:
            return self.roll, self.pitch

    # Keeping compatibility getters
    def get_kalman_roll(self):
        with self.lock:
            return self.roll

    def get_kalman_pitch(self):
        with self.lock:
            return self.pitch
