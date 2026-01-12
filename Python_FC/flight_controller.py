import time
from imu_reader import get_roll_pitch
from pid_controller import PIDController
from mixer import mix_custom
from motor_control import MotorController

PINS = [19, 18, 12, 13]

pid_roll = PIDController(kp=3.0, ki=0.0, kd=1.5, output_limit=(-200, 200))
mc = MotorController(PINS)

THROTTLE = 1350   # giữ drone nhẹ nhàng (chưa hover thực)

def flight_loop():
    try:
        while True:
            roll, pitch = get_roll_pitch()

            roll_out = pid_roll.compute(roll)

            m = mix_custom(roll_out, 0, 0, THROTTLE)

            for i in range(4):
                mc.set_motor(i, m[i])

            print(
                f"\rRoll:{roll:+6.2f}° | PID:{roll_out:+7.2f} | "
                f"M:{m[0]}, {m[1]}, {m[2]}, {m[3]}",
                end="",
                flush=True
            )

            time.sleep(0.01)

    except KeyboardInterrupt:
        mc.stop_all()
        print("\nStop motors.")
