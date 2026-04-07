#!/usr/bin/env python3
"""Simple text-based robot visualizer for Butler Delivery system."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import os


class RobotVisualizer(Node):
    """Simple text-based visualizer showing robot position and state."""

    def __init__(self):
        super().__init__("robot_visualizer")

        self.create_subscription(String, "/butler/status", self.status_callback, 10)
        self.create_subscription(String, "/butler/state", self.state_callback, 10)

        self.current_status = "Waiting for orders..."
        self.current_state = "idle"
        self.current_position = {"x": 0.0, "y": 0.0}
        self.current_table = None
        self.pending_orders = []
        self.active_orders = []

        self.get_logger().info("Robot Visualizer started")
        self.timer = self.create_timer(0.5, self.display_robot)

    def status_callback(self, msg: String):
        self.current_status = msg.data

    def state_callback(self, msg: String):
        try:
            import json
            state_data = json.loads(msg.data)
            self.current_state = state_data.get("state", "unknown")
            self.current_position = state_data.get("position", {"x": 0.0, "y": 0.0})
            self.current_table = state_data.get("current_table")
            self.pending_orders = state_data.get("pending_orders", [])
            self.active_orders = [o.get("table") for o in state_data.get("active_orders", []) if o.get("status") not in ["canceled", "delivered", "timeout"]]
        except:
            pass

    def display_robot(self):
        """Display the robot position and status in ASCII art."""
        os.system('clear' if os.name == 'posix' else 'cls')

        print("🤖 BUTLER DELIVERY ROBOT VISUALIZER")
        print("=" * 50)
        print(f"📍 Status: {self.current_status}")
        print(f"🔄 State: {self.current_state}")
        print(f"📍 Position: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f})")
        print(f"🍽️  Current Table: {self.current_table if self.current_table else 'None'}")
        print(f"📋 Pending Orders: {self.pending_orders}")
        print(f"🚀 Active Orders: {self.active_orders}")
        print()

        # ASCII art representation
        self.draw_map()

    def draw_map(self):
        """Draw a simple ASCII map showing robot position."""
        # Define locations
        locations = {
            "home": (0.0, 0.0),
            "kitchen": (1.0, 0.0),
            "table1": (-0.5, 0.5),
            "table2": (-0.5, -0.5),
            "table3": (1.5, -0.5),
        }

        # Create a simple grid
        grid_size = 10
        grid = [['.' for _ in range(grid_size)] for _ in range(grid_size)]

        # Place locations on grid
        for name, (x, y) in locations.items():
            grid_x = int((x + 2) * 2)  # Scale and offset
            grid_y = int((y + 2) * 2)
            if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                if name == "home":
                    grid[grid_y][grid_x] = "🏠"
                elif name == "kitchen":
                    grid[grid_y][grid_x] = "🍳"
                else:
                    grid[grid_y][grid_x] = f"🪑{name[-1]}"

        # Place robot
        robot_x = int((self.current_position["x"] + 2) * 2)
        robot_y = int((self.current_position["y"] + 2) * 2)
        if 0 <= robot_x < grid_size and 0 <= robot_y < grid_size:
            grid[robot_y][robot_x] = "🤖"

        # Print the grid
        print("MAP:")
        for row in grid:
            print(' '.join(row))
        print()

        # Legend
        print("LEGEND:")
        print("🤖 = Robot")
        print("🏠 = Home")
        print("🍳 = Kitchen")
        print("🪑1/🪑2/🪑3 = Tables")
        print()


def main(args=None):
    rclpy.init(args=args)
    visualizer = RobotVisualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
