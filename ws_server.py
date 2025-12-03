#!/usr/bin/env python3
import asyncio
import websockets
import json
import time

from sensor_manager import get_state
from motor_driver import MotorController
from pid_controller import PIDController
from motor_mixer import QuadXMixer

# --- CONFIG ---
MOTOR_PINS = [19, 18, 12, 13]
HOST = "0.0.0.0"
PORT = 8765

BASE_THROTTLE = 1000
MIN_CMD = 1000
MAX_CMD = 1900

TARGET_ROLL = 0.0
TARGET_PITCH = 0.0

CONTROL_ALLOWED = False
PID_RUNNING = False
STATUS_MSG = "Waiting..."

# PID readiness (chỉ cho START khi GUI đã set đủ)
PID_READY_ROLL = False
PID_READY_PITCH = False

# --- Motors ---
try:
    motors = MotorController(MOTOR_PINS)
except Exception:
    motors = None

mixer = QuadXMixer(min_cmd=MIN_CMD, max_cmd=MAX_CMD)

# --- PID 1 vòng: KHÔNG SET SẴN (0,0,0) ---
pid_roll  = PIDController(kp=0.0, ki=0.0, kd=0.0, i_limit=200, out_limit=400)
pid_pitch = PIDController(kp=0.0, ki=0.0, kd=0.0, i_limit=200, out_limit=400)

def reset_pids():
    pid_roll.reset()
    pid_pitch.reset()

def pid_ready():
    return PID_READY_ROLL and PID_READY_PITCH


async def run_auto_arm():
    global CONTROL_ALLOWED, STATUS_MSG
    CONTROL_ALLOWED = False
    STATUS_MSG = "Auto-Arming: Wait 10s..."
    print(STATUS_MSG)
    await asyncio.sleep(10.0)

    if not motors:
        STATUS_MSG = "Sim Mode (no motors)."
        CONTROL_ALLOWED = True
        reset_pids()
        return

    STATUS_MSG = "Init ESC (0)..."
    motors.set_all_zero()
    await asyncio.sleep(2.0)

    STATUS_MSG = "Sending Min (1000)..."
    motors.set_all(1000)
    await asyncio.sleep(3.0)

    reset_pids()
    STATUS_MSG = "ARMED. Please set PID from GUI."
    print("ARMED.")
    CONTROL_ALLOWED = True


async def handler(websocket):
    global PID_RUNNING, STATUS_MSG, CONTROL_ALLOWED, BASE_THROTTLE, TARGET_ROLL, TARGET_PITCH
    global PID_READY_ROLL, PID_READY_PITCH

    print(f"[CLIENT] {websocket.remote_address}")
    asyncio.create_task(run_auto_arm())

    current_motor_vals = [1000, 1000, 1000, 1000]
    last_t = time.monotonic()

    try:
        while True:
            # dt clamp
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            if dt < 0.001: dt = 0.001
            if dt > 0.05:  dt = 0.05

            # 1) read angles
            st = get_state()
            curr_r = float(st.get("roll", 0.0))
            curr_p = float(st.get("pitch", 0.0))
            curr_y = float(st.get("yaw_rel", 0.0))

            # 2) PID (only when ready)
            r_out = 0.0
            p_out = 0.0
            y_out = 0.0

            if CONTROL_ALLOWED and PID_RUNNING and pid_ready():
                r_out = pid_roll.compute(TARGET_ROLL, curr_r, dt=dt)
                p_out = pid_pitch.compute(TARGET_PITCH, curr_p, dt=dt)

                vals = mixer.compute(BASE_THROTTLE, r_out, p_out, y_out)
                current_motor_vals = vals

                if motors:
                    for i, v in enumerate(vals):
                        motors.set_motor(i, v)

            # 3) telemetry
            payload = {
                "roll": curr_r,
                "pitch": curr_p,
                "yaw": curr_y,

                "armed": CONTROL_ALLOWED,
                "pid_active": PID_RUNNING,
                "msg": STATUS_MSG,

                "pid_roll": r_out,
                "pid_pitch": p_out,
                "pid_yaw": 0.0,

                "motors": current_motor_vals,
                "throttle": BASE_THROTTLE,

                # expose PID current values so GUI can display them
                "pid_cfg": {
                    "roll":  {"kp": pid_roll.kp,  "ki": pid_roll.ki,  "kd": pid_roll.kd,  "ready": PID_READY_ROLL},
                    "pitch": {"kp": pid_pitch.kp, "ki": pid_pitch.ki, "kd": pid_pitch.kd, "ready": PID_READY_PITCH},
                }
            }
            await websocket.send(json.dumps(payload))

            # 4) receive commands
            try:
                msg = await asyncio.wait_for(websocket.recv(), timeout=0.005)
                data = json.loads(msg)
                ctype = data.get("type")

                if ctype == "START_PID":
                    if not CONTROL_ALLOWED:
                        STATUS_MSG = "Wait for Arming..."
                    elif not pid_ready():
                        STATUS_MSG = "Set PID (Roll & Pitch) from GUI first!"
                    else:
                        PID_RUNNING = True
                        reset_pids()
                        STATUS_MSG = f"PID ON (Thr: {BASE_THROTTLE})"

                elif ctype == "STOP_PID":
                    PID_RUNNING = False
                    reset_pids()
                    STATUS_MSG = "PID STOPPED."
                    current_motor_vals = [1000, 1000, 1000, 1000]
                    if motors:
                        motors.set_all(1000)

                elif ctype == "set_throttle":
                    val = int(data.get("value", 1000))
                    BASE_THROTTLE = max(1000, min(1800, val))
                    STATUS_MSG = f"Throttle: {BASE_THROTTLE}"

                elif ctype == "set_target":
                    TARGET_ROLL = float(data.get("roll", 0.0))
                    TARGET_PITCH = float(data.get("pitch", 0.0))
                    STATUS_MSG = f"Target R={TARGET_ROLL:.1f} P={TARGET_PITCH:.1f}"

                elif ctype == "set_pid":
                    ax = data.get("axis")
                    kp = float(data.get("kp", 0.0))
                    ki = float(data.get("ki", 0.0))
                    kd = float(data.get("kd", 0.0))

                    if ax == "roll":
                        pid_roll.update_constants(kp, ki, kd)
                        PID_READY_ROLL = True
                        STATUS_MSG = f"SET ROLL PID OK"
                    elif ax == "pitch":
                        pid_pitch.update_constants(kp, ki, kd)
                        PID_READY_PITCH = True
                        STATUS_MSG = f"SET PITCH PID OK"
                    else:
                        STATUS_MSG = "Axis not supported (yaw later)."

                elif ctype == "test_motor":
                    # Manual test only when PID OFF
                    if CONTROL_ALLOWED and (not PID_RUNNING) and motors:
                        mid = int(data.get("motor_id", 1)) - 1
                        val_percent = float(data.get("value", 0.0))
                        mid = max(0, min(3, mid))
                        val_percent = max(0.0, min(100.0, val_percent))
                        pulse = 1000 + int(val_percent * 10)
                        motors.set_motor(mid, pulse)
                        current_motor_vals[mid] = pulse
                        STATUS_MSG = f"TEST M{mid+1}={pulse}"

            except asyncio.TimeoutError:
                pass
            except websockets.exceptions.ConnectionClosed:
                break

            await asyncio.sleep(0.01)

    except Exception as e:
        print(f"Err: {e}")
    finally:
        PID_RUNNING = False
        CONTROL_ALLOWED = False
        reset_pids()
        if motors:
            motors.stop_all()
        print("Disconnected.")


async def main():
    print(f"Server on {HOST}:{PORT}")
    async with websockets.serve(handler, HOST, PORT):
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        if motors:
            motors.stop_all()
        print("Stopped.")
