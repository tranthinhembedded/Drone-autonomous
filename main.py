from sensor_manager import get_state
import time

def main():
    print("=== DRONE MAIN: READ FULL IMU ===")

    try:
        while True:
            st = get_state()

            line = (
                f"Roll:{st['roll']:+7.2f}°  "
                f"Pitch:{st['pitch']:+7.2f}°  "
                f"Yaw:{st['yaw_rel']:+7.2f}°  "
            )

            print("\r" + line.ljust(70), end="", flush=True)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStop.")

if __name__ == "__main__":
    main()
