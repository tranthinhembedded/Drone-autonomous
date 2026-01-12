# pid_controller.py
import time


class PIDController:
    """
    PID for ANGLE control (outer-loop style):
    - P, I on angle error
    - D on measurement (prevents derivative kick)
    - D low-pass filter (reduces noise)
    - Conditional integration anti-windup (prevents I building when saturated)

    NOTE: PID gains are expected to be set from GUI at runtime.
    Defaults are zero.
    """

    def __init__(
        self,
        kp=0.0,
        ki=0.0,
        kd=0.0,
        i_limit=0.0,          # 0 => no clamp
        out_limit=0.0,        # 0 => no clamp
        d_tau=0.02,           # D-term LPF time constant (seconds). 0.01~0.05 typical on Pi
        dt_min=0.001,
        dt_max=0.05,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.i_limit = float(i_limit)
        self.out_limit = float(out_limit)

        self.d_tau = float(d_tau)
        self.dt_min = float(dt_min)
        self.dt_max = float(dt_max)

        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_current = None
        self.prev_time = None

        self.d_filt = 0.0
        self.last_output = 0.0

    @staticmethod
    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def compute(self, target, current, dt=None):
        now = time.monotonic()

        # dt handling: if dt is supplied by caller, use it; otherwise compute internally
        if dt is None:
            if self.prev_time is None:
                self.prev_time = now
                self.prev_current = float(current)
                self.prev_error = float(target) - float(current)
                self.d_filt = 0.0
                self.last_output = 0.0
                return 0.0
            dt = now - self.prev_time
            self.prev_time = now

        dt = float(dt)
        dt = self._clamp(dt, self.dt_min, self.dt_max)

        target = float(target)
        current = float(current)
        error = target - current

        # Prime previous states on first valid compute call
        if self.prev_error is None:
            self.prev_error = error
        if self.prev_current is None:
            self.prev_current = current

        # --- P ---
        p = self.kp * error

        # --- D (ON MEASUREMENT) ---
        # d(current)/dt, then negate so that increasing current acts like decreasing error
        d_meas = (current - self.prev_current) / dt
        d_raw = -self.kd * d_meas

        # Low-pass filter D
        # alpha = dt/(tau+dt). tau=0 => no filter
        if self.d_tau > 0.0:
            alpha = dt / (self.d_tau + dt)
            self.d_filt += alpha * (d_raw - self.d_filt)
            d = self.d_filt
        else:
            d = d_raw

        # --- I (TRAPEZOID) with conditional integration anti-windup ---
        i_new = self.integral
        if self.ki != 0.0:
            i_new = self.integral + 0.5 * (error + self.prev_error) * dt

            # Clamp integral state (in raw integral units, before multiplying by ki)
            if self.i_limit and self.i_limit > 0:
                i_new = self._clamp(i_new, -self.i_limit, self.i_limit)

        i_term = self.ki * i_new

        # Pre-sat output
        u_pre = p + i_term + d

        # Saturate output if needed
        u = u_pre
        saturated = False
        if self.out_limit and self.out_limit > 0:
            u = self._clamp(u_pre, -self.out_limit, self.out_limit)
            saturated = (u != u_pre)

        # Conditional integration:
        # If saturated, only accept integral update if it helps *unsaturate* the output.
        accept_i = True
        if saturated and self.ki != 0.0:
            if (u_pre > self.out_limit and error > 0) or (u_pre < -self.out_limit and error < 0):
                accept_i = False

        if accept_i:
            self.integral = i_new  # commit

        # Update previous samples
        self.prev_error = error
        self.prev_current = current
        self.last_output = u

        return u
