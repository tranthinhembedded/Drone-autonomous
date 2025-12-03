import asyncio
import websockets
import json
import time
import math

from sensor_manager import SensorManager
from pid_controller import PIDController
from motor_mixer import mix_quad_x

# Throttle base
BASE_THROTTLE = 1000
MAX_THROTTLE = 2000
MIN_THROTTLE = 1000

# PID target (degrees)
TARGET_ROLL = 0.0
TARGET_PITCH = 0.0

# WS server config
HOST = "0.0.0.0"
PORT = 8765

# Shared state
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

# Init sensor manager
sensor = SensorManager()

# PID controllers (gains set by GUI)
pid_roll = PIDController(out_limit=400, i_limit=200)  # kp/ki/kd will be overwritten by GUI
pid_pitch = PIDController(out_limit=400, i_limit=200)

def apply_pid_values():
    pid_roll.kp = state["pid"]["roll"]["kp"]
    pid_roll.ki = state["pid"]["roll"]["ki"]
    pid_roll.kd = state["pid"]["roll"]["kd"]

    pid_pitch.kp = state["pid"]["pitch"]["kp"]
    pid_pitch.ki = state["pid"]["pitch"]["ki"]
    pid_pitch.kd = state["pid"]["pitch"]["kd"]

async def handler(websocket):
    print("[WS] Client connected")
    state["connected"] = True

    # control loop timing
    last_t = time.time()

    try:
        async for message in websocket:
            # Parse message
            try:
                data = json.loads(message)
            except Exception:
                continue

            cmd = data.get("cmd", "")

            if cmd == "PID":
                # Update PID values from GUI
                roll = data.get("roll", {})
                pitch = data.get("pitch", {})

                state["pid"]["roll"]["kp"] = float(roll.get("kp", 0.0))
                state["pid"]["roll"]["ki"] = float(roll.get("ki", 0.0))
                state["pid"]["roll"]["kd"] = float(roll.get("kd", 0.0))

                state["pid"]["pitch"]["kp"] = float(pitch.get("kp", 0.0))
                state["pid"]["pitch"]["ki"] = float(pitch.get("ki", 0.0))
                state["pid"]["pitch"]["kd"] = float(pitch.get("kd", 0.0))

                apply_pid_values()

            elif cmd == "THROTTLE":
                state["throttle"] = int(data.get("throttle", BASE_THROTTLE))
                state["throttle"] = max(MIN_THROTTLE, min(MAX_THROTTLE, state["throttle"]))

            elif cmd == "ARM":
                state["armed"] = True
                state["pid_enabled"] = False
                pid_roll.reset()
                pid_pitch.reset()
                state["motors"] = [MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE]
                print("[WS] ARMED")

            elif cmd == "DISARM":
                state["armed"] = False
                state["pid_enabled"] = False
                pid_roll.reset()
                pid_pitch.reset()
                state["motors"] = [MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE]
                print("[WS] DISARMED")

            elif cmd == "START":
                # Start PID
                # Require that GUI has set non-zero kp at least
                if state["pid"]["roll"]["kp"] == 0.0 and state["pid"]["pitch"]["kp"] == 0.0:
                    print("[WS] START ignored: PID not set from GUI")
                else:
                    state["pid_enabled"] = True
                    pid_roll.reset()
                    pid_pitch.reset()
                    last_t = time.time()
                    print("[WS] PID STARTED")

            elif cmd == "STOP":
                state["pid_enabled"] = False
                pid_roll.reset()
                pid_pitch.reset()
                state["motors"] = [MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE]
                print("[WS] PID STOPPED")

            # --- Control loop tick (triggered per message) ---
            now = time.time()
            dt = now - last_t
            last_t = now

            # clamp dt
            dt = max(0.001, min(0.05, dt))

            # Read sensors
            roll, pitch = sensor.get_roll_pitch()
            state["angles"]["roll"] = roll
            state["angles"]["pitch"] = pitch

            # Compute PID outputs
            out_roll = 0.0
            out_pitch = 0.0
            if state["armed"] and state["pid_enabled"]:
                out_roll = pid_roll.compute(TARGET_ROLL, roll, dt=dt)
                out_pitch = pid_pitch.compute(TARGET_PITCH, pitch, dt=dt)

            # Mix motors
            if state["armed"]:
                m1, m2, m3, m4 = mix_quad_x(
                    throttle=state["throttle"],
                    roll=out_roll,
                    pitch=out_pitch,
                    yaw=0.0,
                    min_pwm=MIN_THROTTLE,
                    max_pwm=MAX_THROTTLE,
                )
                state["motors"] = [m1, m2, m3, m4]
            else:
                state["motors"] = [MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE]

            # Send telemetry back
            await websocket.send(json.dumps(state))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print("[WS] Client disconnected")
        state["connected"] = False
        state["armed"] = False
        state["pid_enabled"] = False
        state["motors"] = [MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE]

async def main():
    print(f"[WS] Starting server on {HOST}:{PORT}")
    async with websockets.serve(handler, HOST, PORT):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
