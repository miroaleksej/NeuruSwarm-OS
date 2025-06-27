#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bosdyn.api import spot_pb2
from bosdyn.client import create_standard_sdk, ResponseError
from ns_os_interfaces.msg import SpotCommand, SpotStatus

class SpotAdapter(Node):
    def __init__(self):
        super().__init__('spot_adapter')
        
        # Initialize Spot SDK
        self.sdk = create_standard_sdk('SpotAdapter')
        self.robot = None
        self.connected = False
        
        # ROS 2 interfaces
        self.command_sub = self.create_subscription(
            SpotCommand, '/ns/spot_command', self.command_callback, 10)
        self.status_pub = self.create_publisher(
            SpotStatus, '/ns/spot_status', 10)
            
        # Connect to Spot
        self.connect_to_spot()

    def connect_to_spot(self):
        try:
            self.robot = self.sdk.create_robot('spot-robot')
            self.robot.authenticate('username', 'password')
            self.robot.time_sync.wait_for_sync()
            self.connected = True
            self.get_logger().info("Successfully connected to Spot")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Spot: {str(e)}")
            self.connected = False

    def command_callback(self, msg):
        if not self.connected:
            self.get_logger().warn("Spot not connected, ignoring command")
            return
            
        try:
            if msg.command == "walk":
                self._handle_walk_command(msg)
            elif msg.command == "grasp":
                self._handle_grasp_command(msg)
            # Add other commands as needed
            
            # Publish status
            status = SpotStatus()
            status.status = "Command executed successfully"
            self.status_pub.publish(status)
            
        except ResponseError as e:
            self.get_logger().error(f"Spot command failed: {str(e)}")
            status = SpotStatus()
            status.status = f"Error: {str(e)}"
            self.status_pub.publish(status)

    def _handle_walk_command(self, msg):
        # Implementation for walk command
        pass

    def _handle_grasp_command(self, msg):
        # Implementation for grasp command
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SpotAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
