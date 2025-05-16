import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode
import time
import math

class DroneKitNode(Node):
    def __init__(self):
        super().__init__('dronekit_node')
        self.get_logger().info('Connecting to drone...')
        
        # Connect to the drone using DroneKit
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

        self.get_logger().info('Connected!')

    def arm_and_takeoff(self, altitude):
        """Arms the drone and takes off to the specified altitude."""
        self.get_logger().info('Arming motors...')
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            self.get_logger().info('Waiting for arming...')
            time.sleep(1)

        self.get_logger().info('Taking off...')
        self.vehicle.simple_takeoff(altitude)

        while True:
            self.get_logger().info(f'Altitude: {self.vehicle.location.global_relative_frame.alt}')
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                self.get_logger().info(f'Reached target altitude! ({altitude}m)')
                break
            time.sleep(1)

    def set_velocity(self, vx, vy, vz):
        """Set velocity (vx: forward/backward, vy: right/left, vz: up/down)."""
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,  
            1,  # ✅ MAV_FRAME_BODY_NED for movement relative to the drone
            0b0000111111000111,  # type_mask (ignore everything except velocity)
            0, 0, 0,  
            vx, vy, vz,  
            0, 0, 0,  
            0, 0  
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def move_forward(self, speed=1.0, duration=2):
        """Moves the drone forward."""
        self.get_logger().info('Moving forward')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.set_velocity(speed, 0, 0)
            time.sleep(0.1)
        self.set_velocity(0, 0, 0)

    def move_backward(self, speed=1.0, duration=2):
        """Moves the drone backward."""
        self.get_logger().info('Moving backward')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.set_velocity(-speed, 0, 0)
            time.sleep(0.1)
        self.set_velocity(0, 0, 0)

    def move_left(self, speed=1.0, duration=2):
        """Moves the drone left."""
        self.get_logger().info('Moving left')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.set_velocity(0, -speed, 0)
            time.sleep(0.1)
        self.set_velocity(0, 0, 0)

    def move_right(self, speed=1.0, duration=2):
        """Moves the drone right."""
        self.get_logger().info('Moving right')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.set_velocity(0, speed, 0)
            time.sleep(0.1)
        self.set_velocity(0, 0, 0)

    def get_current_yaw(self):
        """Returns the current yaw angle."""
        if self.vehicle.attitude is not None:
            return (math.degrees(self.vehicle.attitude.yaw) + 360) % 360
        return None

    def set_yaw(self, target_yaw, speed=10):
        """Rotates the drone to an absolute yaw angle."""
        self.get_logger().info(f"Setting yaw to {target_yaw} degrees")
        self.vehicle.mode = VehicleMode("GUIDED")

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  
            115,  # MAV_CMD_CONDITION_YAW
            0,  
            target_yaw,  
            speed,  
            1,  
            1,  
            0, 0, 0  
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        while True:
            current_yaw = self.get_current_yaw()
            if current_yaw is not None:
                error = abs(target_yaw - current_yaw)
                self.get_logger().info(f"Current yaw: {current_yaw:.2f} degrees, Error: {error:.2f}")
                if error < 2:
                    self.get_logger().info("Reached target yaw!")
                    break
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = DroneKitNode()

    node.arm_and_takeoff(1)  # Takeoff to 10m
    
    node.move_forward()
    time.sleep(1)

    node.set_yaw(90)  # Rotate 90 degrees
    time.sleep(1)

    node.move_forward()
    time.sleep(1)

    node.set_yaw(180)  # Rotate 180 degrees
    time.sleep(1)

    node.move_forward()
    time.sleep(1)

    node.get_logger().info("Landing...")
    node.vehicle.mode = VehicleMode("LAND")
    time.sleep(5)

    node.vehicle.armed = False
    node.vehicle.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
