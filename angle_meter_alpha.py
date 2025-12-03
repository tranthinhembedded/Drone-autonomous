# angle_meter_alpha.py
from kalman_filter import KalmanAngle
import smbus2
import time
import math
import threading

class AngleMeterAlpha:
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self.DeviceAddress = 0x68
        self.MPU_Init()
        
        self.pitch = 0.0
        self.roll = 0.0
        
        # [MỚI] Các biến lưu tốc độ góc cho Rate Loop PID
        self.rate_roll = 0.0
        self.rate_pitch = 0.0
        self.rate_yaw = 0.0
        
        self.compl_pitch = 0.0
        self.compl_roll = 0.0
        self.kalman_pitch = 0.0
        self.kalman_roll = 0.0

    def MPU_Init(self):
        self.bus.write_byte_data(self.DeviceAddress, 0x19, 7)
        self.bus.write_byte_data(self.DeviceAddress, 0x6B, 1)
        self.bus.write_byte_data(self.DeviceAddress, 0x1A, int('0000110', 2))
        self.bus.write_byte_data(self.DeviceAddress, 0x1B, 24)
        self.bus.write_byte_data(self.DeviceAddress, 0x38, 1)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.DeviceAddress, addr)
        low = self.bus.read_byte_data(self.DeviceAddress, addr+1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    def measure(self):
        angleThread = threading.Thread(target=self.measureAngles)
        angleThread.daemon = True # Tự tắt khi chương trình chính tắt
        angleThread.start()

    def measureAngles(self):
        kalmanX = KalmanAngle()
        kalmanY = KalmanAngle()

        RestrictPitch = True
        radToDeg = 57.2957786
        kalAngleX = 0
        kalAngleY = 0

        # MPU6050 Registers
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H = 0x43
        GYRO_YOUT_H = 0x45
        GYRO_ZOUT_H = 0x47

        time.sleep(1)
        
        # Init Timer
        timer = time.time()
        
        # Init Complementary values
        accX = self.read_raw_data(ACCEL_XOUT_H)
        accY = self.read_raw_data(ACCEL_YOUT_H)
        accZ = self.read_raw_data(ACCEL_ZOUT_H)
        
        if (RestrictPitch):
            roll = math.atan2(accY, accZ) * radToDeg
            pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
        else:
            roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
            pitch = math.atan2(-accX, accZ) * radToDeg

        kalmanX.setAngle(roll)
        kalmanY.setAngle(pitch)
        compAngleX = roll
        compAngleY = pitch

        while True:
            try:
                accX = self.read_raw_data(ACCEL_XOUT_H)
                accY = self.read_raw_data(ACCEL_YOUT_H)
                accZ = self.read_raw_data(ACCEL_ZOUT_H)
                gyroX = self.read_raw_data(GYRO_XOUT_H)
                gyroY = self.read_raw_data(GYRO_YOUT_H)
                gyroZ = self.read_raw_data(GYRO_ZOUT_H)

                dt = time.time() - timer
                timer = time.time()

                if (RestrictPitch):
                    roll = math.atan2(accY,accZ) * radToDeg
                    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
                else:
                    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
                    pitch = math.atan2(-accX,accZ) * radToDeg

                # [QUAN TRỌNG] Tính tốc độ góc (Rate)
                # 131.0 là độ nhạy của Gyro ở mức +-250dps
                gyroXRate = gyroX / 131.0
                gyroYRate = gyroY / 131.0
                gyroZRate = gyroZ / 131.0 

                # Cập nhật Kalman
                if (RestrictPitch):
                    if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                        kalmanX.setAngle(roll)
                        compAngleX = roll
                        kalAngleX   = roll
                    else:
                        kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

                    if(abs(kalAngleY)>90):
                        gyroYRate  = -gyroYRate
                        kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
                    else:
                        kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
                else:
                    if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
                        kalmanY.setAngle(pitch)
                        compAngleY = pitch
                        kalAngleY   = pitch
                    else:
                        kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)
                    if(abs(kalAngleX)>90):
                        gyroXRate  = -gyroXRate
                        kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

                # Complementary Filter
                compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
                compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

                # --- LƯU GIÁ TRỊ VÀO BIẾN CLASS ĐỂ GỌI TỪ NGOÀI ---
                self.pitch = compAngleY
                self.roll  = compAngleX
                
                self.kalman_pitch = kalAngleY
                self.kalman_roll = kalAngleX
                self.compl_pitch = compAngleY
                self.compl_roll = compAngleX
                
                # [MỚI] Lưu Rate để dùng cho PID vòng trong
                self.rate_roll = gyroXRate
                self.rate_pitch = gyroYRate
                self.rate_yaw = gyroZRate

                time.sleep(0.004) # Tăng tốc độ đọc lên (4ms ~ 250Hz)

            except Exception as exc:
                print(f"IMU Error: {exc}")

    # --- GETTERS ---
    def get_complementary_roll(self): return self.compl_roll
    def get_complementary_pitch(self): return self.compl_pitch
    
    # [MỚI] Hàm lấy tốc độ góc cho PID
    def get_rates(self):
        return self.rate_roll, self.rate_pitch, self.rate_yaw