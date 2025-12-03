# pid_controller.py
import time

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class PIDController:
    """
    PID cổ điển 1 vòng:
    - P = kp*e
    - I = I + ki*(e+e_prev)*dt/2  (trapezoid, giống firmware)
    - D = kd*(e-e_prev)/dt
    - Clamp I-term và output
    - dt clamp chống giật
    """

    def __init__(
        self,
        kp=0.0, ki=0.0, kd=0.0,
        i_limit=400.0,
        out_limit=400.0,
        dt_limits=(0.001, 0.05),
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.i_limit = float(i_limit)
        self.out_limit = float(out_limit)
        self.dt_min, self.dt_max = float(dt_limits[0]), float(dt_limits[1])

        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_t = time.monotonic()
        self._primed = False

    def update_constants(self, kp, ki, kd):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

    def compute(self, target, current, dt=None):
        now = time.monotonic()
        if dt is None:
            dt = now - self.last_t
        self.last_t = now
        dt = clamp(dt, self.dt_min, self.dt_max)

        error = float(target) - float(current)

        # prime lần đầu để tránh D spike
        if not self._primed:
            self.prev_error = error
            self._primed = True

        # P
        p_term = self.kp * error

        # I trapezoid
        if self.ki != 0.0:
            self.integral += self.ki * (error + self.prev_error) * (dt * 0.5)
            self.integral = clamp(self.integral, -self.i_limit, self.i_limit)

        # D classic
        d_term = 0.0
        if self.kd != 0.0:
            d_term = self.kd * ((error - self.prev_error) / dt)

        out = p_term + self.integral + d_term
        out = clamp(out, -self.out_limit, self.out_limit)

        self.prev_error = error
        return out
