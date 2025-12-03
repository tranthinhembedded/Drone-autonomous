# imu_reader.py
from angle_meter_alpha import AngleMeterAlpha
import time

# Khởi tạo đối tượng IMU một lần duy nhất
print("Initializing IMU...")
imu = AngleMeterAlpha()
imu.measure() # Bắt đầu luồng đọc cảm biến ngầm
print("IMU Reading started.")

def get_roll_pitch():
    """
    Trả về góc nghiêng (Angle) dùng cho PID Vòng Ngoài (Stabilize)
    """
    # Sử dụng bộ lọc bù (Complementary) vì nó mượt và ít trễ hơn Kalman trong code này
    # Lưu ý: Kiểm tra thực tế xem Roll là X hay Y. 
    # Trong code gốc: compAngleX gán cho roll, compAngleY gán cho pitch.
    return imu.get_kalman_pitch(), imu.get_kalman_roll()