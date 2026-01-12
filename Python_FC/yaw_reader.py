from yaw_meter_alpha import YawMeterAlpha

yaw = YawMeterAlpha()

def get_yaw_raw():
    return yaw.get_raw_yaw()

def get_yaw_kalman():
    return yaw.get_kalman_yaw()

def get_yaw_relative():
    return yaw.get_relative_yaw()

