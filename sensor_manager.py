# sensor_manager.py
from imu_reader import get_roll_pitch

class SensorManager:
    def __init__(self):
        pass

    def get_roll_pitch(self):
        return get_roll_pitch()
