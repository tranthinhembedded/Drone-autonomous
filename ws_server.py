# ws_server.py
import asyncio
import json
import time

import websockets

from sensor_manager import SensorManager
from pid_controller import PIDController
from motor_mixer import mix_quad_x

# PWM limits
BASE_THROTTLE = 1000
MAX_THROTTLE = 2000
MIN_THROTTLE = 1000

# PID targets (degrees)
TARGET_ROLL = 0.0
TARGET_PITCH = 0.0

# WS server config
HOST = "0.0.0.0"
PORT = 8765

# Internal state
state = {
    "connected": False,
    "armed": False,
    "pid_enabled": False,
    "throttle": BASE_THROTTLE,
    "pid": {
        "roll": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
        "pitch": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
        "yaw": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
    },
    "angles": {"roll": 0.0, "pitch": 0.0},
    "motors": [MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE],
}

# Sensors
sensor = SensorManager()

# PID controllers (gains set by GUI at runtime)
pid_roll = PIDController(out_limit=400, i_limit=200)   # kp/ki/kd overwritten by GUI
pid_pitch = PIDController(out_limit=400, i_limit=200)

# Optional: direct motor override (test_motor)
motor_override = None  # None or dict {"motor_id": int, "value": int}


def apply_pid_values():
    # Roll
    pid_roll.kp = float(state["pid"]["roll"]["kp"])
    pid_roll.ki = float(state["pid"]["roll"]["ki"])
    pid_roll.kd = float(state["pid"]["roll"]["kd"])
    # Pitch
    pid_pitch.kp = float(state["pid"]["pitch"]["kp"])
    pid_pitch.ki = float(state["pid"]["pitch"]["ki"])
    pid_pitch.kd = float(state["pid"]["pitch"]["kd"])


def reset_control():
    pid_roll.reset()
    pid_pitch.reset()
    state["pid_enabled"] = False
    state["motors"] = [MIN_THROTTLE] * 4


def make_gui_payload(msg=""):
    # GUI expects flattened keys in gui_modern_sync.py
    return {
        "roll": state["angles"]["roll"],
        "pitch": state["angles"]["pitch"],
        "yaw": 0.0,
        "armed": state["armed"],
        "pid_active": state["pid_enabled"],
        "throttle": state["throttle"],
        "motors": state["motors"],
        "msg": msg,
    }


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


async def handler(websocket):
    global motor_override

    print("[WS] Client connected")
    state["connected"] = True

    # AUTO-ARM on successful GUI connection (per your requirement)
    state["armed"] = True
    reset_control()

    last_t = time.time()

    try:
        async for message in websocket:
            # Parse message
            try:
                data = json.loads(message)
            except Exception:
                await websocket.send(json.dumps(make_gui_payload("Bad JSON")))
                continue

            # Support BOTH protocols:
            # - Old: {"cmd":"THROTTLE", ...}
            # - GUI(client): {"type":"set_throttle", ...}, {"type":"START_PID"}, ...
            cmd = data.get("cmd")
            msg_type = data.get("type")

            # ----------------------------
            # Handle incoming commands
            # ----------------------------

            # Set PID (GUI): {"type":"set_pid","axis":"roll","kp":..,"ki":..,"kd":..}
            # Old format: {"cmd":"PID","roll":{...},"pitch":{...}}
            if cmd == "PID" or msg_type == "set_pid":
                if msg_type == "set_pid":
                    ax = data.get("axis")
                    if ax in state["pid"]:
                        state["pid"][ax]["kp"] = float(data.get("kp", 0.0))
                        state["pid"][ax]["ki"] = float(data.get("ki", 0.0))
                        state["pid"][ax]["kd"] = float(data.get("kd", 0.0))
                        apply_pid_values()
                else:
                    roll = data.get("roll", {})
                    pitch = data.get("pitch", {})

                    state["pid"]["roll"]["kp"] = float(roll.get("kp", 0.0))
                    state["pid"]["roll"]["ki"] = float(roll.get("ki", 0.0))
                    state["pid"]["roll"]["kd"] = float(roll.get("kd", 0.0))

                    state["pid"]["pitch"]["kp"] = float(pitch.get("kp", 0.0))
                    state["pid"]["pitch"]["ki"] = float(pitch.get("ki", 0.0))
                    state["pid"]["pitch"]["kd"] = float(pitch.get("kd", 0.0))

                    apply_pid_values()

            # Set throttle (GUI): {"type":"set_throttle","value":1234}
            # Old format: {"cmd":"THROTTLE","throttle":1234}
            elif cmd == "THROTTLE" or msg_type == "set_throttle":
                val = data.get("throttle", data.get("value", BASE_THROTTLE))
                state["throttle"] = int(val)
                state["throttle"] = int(clamp(state["throttle"], MIN_THROTTLE, MAX_THROTTLE))

            # START PID (GUI): {"type":"START_PID"}
            # Old format: {"cmd":"START"} or {"cmd":"START_PID"}
            elif cmd in ("START", "START_PID") or msg_type == "START_PID":
                # Require that GUI has set some kp (same rule as your old code)
                if state["pid"]["roll"]["kp"] == 0.0 and state["pid"]["pitch"]["kp"] == 0.0:
                    state["pid_enabled"] = False
                else:
                    state["pid_enabled"] = True
                    pid_roll.reset()
                    pid_pitch.reset()
                    last_t = time.time()

            # STOP PID (GUI): {"type":"STOP_PID"}
            # Old format: {"cmd":"STOP"} or {"cmd":"STOP_PID"}
            elif cmd in ("STOP", "STOP_PID") or msg_type == "STOP_PID":
                reset_control()

            # Optional explicit DISARM/ARM (even if GUI doesn't send)
            elif cmd == "DISARM" or msg_type == "DISARM":
                state["armed"] = False
                motor_override = None
                reset_control()

            elif cmd == "ARM" or msg_type == "ARM":
                state["armed"] = True
                motor_override = None
                reset_control()

            # Motor test (GUI): {"type":"test_motor","motor_id":0..3,"value":pwm}
            elif msg_type == "test_motor" or cmd == "TEST_MOTOR":
                # If you want motor test to bypass PID/mixer temporarily
                try:
                    motor_id = int(data.get("motor_id", -1))
                    value = int(data.get("value", MIN_THROTTLE))
                    value = int(clamp(value, MIN_THROTTLE, MAX_THROTTLE))
                    if 0 <= motor_id <= 3:
                        motor_override = {"motor_id": motor_id, "value": value}
                except Exception:
                    motor_override = None

            # Clear motor override (optional)
            elif msg_type == "clear_motor_test" or cmd == "CLEAR_MOTOR_TEST":
                motor_override = None

            # ----------------------------
            # Control loop tick
            # ----------------------------
            now = time.time()
            dt = now - last_t
            last_t = now
            dt = max(0.001, min(0.05, dt))

            # Read sensors
            roll, pitch = sensor.get_roll_pitch()
            state["angles"]["roll"] = float(roll)
            state["angles"]["pitch"] = float(pitch)

            # If not armed -> stop motors
            if not state["armed"]:
                state["motors"] = [MIN_THROTTLE] * 4
                await websocket.send(json.dumps(make_gui_payload()))
                continue

            # Motor override mode
            if motor_override is not None:
                motors = [MIN_THROTTLE] * 4
                motors[motor_override["motor_id"]] = motor_override["value"]
                state["motors"] = motors
                await websocket.send(json.dumps(make_gui_payload("MOTOR TEST")))
                continue

            # Normal PID/mixer mode
            out_roll = 0.0
            out_pitch = 0.0
            if state["pid_enabled"]:
                out_roll = pid_roll.compute(TARGET_ROLL, roll, dt=dt)
                out_pitch = pid_pitch.compute(TARGET_PITCH, pitch, dt=dt)

            m1, m2, m3, m4 = mix_quad_x(
                throttle=state["throttle"],
                roll=out_roll,
                pitch=out_pitch,
                yaw=0.0,  # yaw disabled for now
                min_pwm=MIN_THROTTLE,
                max_pwm=MAX_THROTTLE,
            )
            state["motors"] = [m1, m2, m3, m4]

            # Send telemetry back to GUI
            await websocket.send(json.dumps(make_gui_payload()))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print("[WS] Client disconnected")
        state["connected"] = False
        state["armed"] = False
        state["pid_enabled"] = False
        motor_override = None
        state["motors"] = [MIN_THROTTLE] * 4


async def main():
    print(f"[WS] Starting server on {HOST}:{PORT}")
    async with websockets.serve(handler, HOST, PORT):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
