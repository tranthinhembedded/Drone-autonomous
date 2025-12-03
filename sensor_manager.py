# sensor_manager.py
from imu_reader import get_roll_pitch

# Yaw: optional (tạm chưa dùng cho điều khiển)
try:
    from yaw_reader import get_yaw_relative
except Exception:
    get_yaw_relative = None


def get_state():
    roll, pitch = get_roll_pitch()

    yaw_rel = 0.0
    if get_yaw_relative is not None:
        try:
            yaw_rel = float(get_yaw_relative())
        except Exception:
            yaw_rel = 0.0

    return {
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw_rel": float(yaw_rel),
    }
