from dronekit import connect, VehicleMode
import time

def main():
    print("🔌 Connecting to Pixhawk...")
    vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200)

    # Try setting mode to GUIDED
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
    print(f"✅ Mode set to: {vehicle.mode.name}")

    vehicle.armed = True

if __name__ == "__main__":
    main()
