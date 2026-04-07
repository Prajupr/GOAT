#!/usr/bin/env python3
"""Interactive client for the Butler Delivery system."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32, String
from std_srvs.srv import Empty
from typing import List


class InteractiveClient(Node):
    """Terminal-based interactive client for sending orders and confirmations."""

    def __init__(self):
        super().__init__("butler_delivery_interactive_client")

        self.order_publisher = self.create_publisher(
            Int32MultiArray,
            "/butler/orders",
            10
        )

        self.cancel_publisher = self.create_publisher(
            Int32,
            "/butler/cancel_order",
            10
        )

        self.kitchen_client = self.create_client(Empty, "/butler/confirm_at_kitchen")
        self.table_client = self.create_client(Empty, "/butler/confirm_at_table")

        self.create_subscription(String, "/butler/status", self.status_callback, 10)
        self.create_subscription(String, "/butler/state", self.state_callback, 10)

        self.latest_status = "(not received yet)"
        self.latest_state = "(not received yet)"

    def status_callback(self, msg: String):
        self.latest_status = msg.data

    def state_callback(self, msg: String):
        self.latest_state = msg.data

    def publish_order(self, table_ids: List[int]):
        msg = Int32MultiArray()
        msg.data = table_ids
        self.order_publisher.publish(msg)
        self.get_logger().info(f"Published order for tables: {table_ids}")

    def call_service(self, client, name: str):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {name} is unavailable")
            return False

        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error(f"Service call failed: {name}")
            return False

        self.get_logger().info(f"Service called: {name}")
        return True

    def print_help(self):
        print("\nButler Delivery interactive client commands:")
        print("  order <table1> [table2 ...]    - Send one or more table orders")
        print("  confirm_kitchen                - Confirm the kitchen has food ready")
        print("  confirm_table                 - Confirm food delivery at current table")
        print("  cancel [table_id]              - Cancel the active order or specific table")
        print("  status                        - Show latest butler status")
        print("  state                         - Show latest butler state")
        print("  help                          - Show this menu")
        print("  exit                          - Quit this client\n")

    def run(self):
        self.get_logger().info("Interactive client initialized. Type 'help' for commands.")
        self.print_help()

        while rclpy.ok():
            try:
                text = input("butler> ").strip()
            except EOFError:
                break

            if not text:
                continue

            args = text.split()
            cmd = args[0].lower()

            if cmd in ["exit", "quit"]:
                break
            elif cmd == "help":
                self.print_help()
            elif cmd == "order":
                if len(args) < 2:
                    self.get_logger().info("Usage: order <table1> [table2 ...]")
                    continue
                try:
                    table_ids = [int(value) for value in args[1:]]
                except ValueError:
                    self.get_logger().error("Table IDs must be integers")
                    continue
                self.publish_order(table_ids)
            elif cmd == "confirm_kitchen":
                self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")
            elif cmd == "confirm_table":
                self.call_service(self.table_client, "/butler/confirm_at_table")
            elif cmd == "cancel":
                cancel_id = -1
                if len(args) == 2:
                    try:
                        cancel_id = int(args[1])
                    except ValueError:
                        self.get_logger().error("Table ID must be an integer")
                        continue
                cancel_msg = Int32()
                cancel_msg.data = cancel_id
                self.cancel_publisher.publish(cancel_msg)
                self.get_logger().info(f"Cancel request sent for table {cancel_id if cancel_id >= 0 else 'current active order'}")
            elif cmd == "status":
                print(f"Latest status: {self.latest_status}")
            elif cmd == "state":
                print(f"Latest state: {self.latest_state}")
            else:
                self.get_logger().info(f"Unknown command: {cmd}. Type 'help' to list commands.")

        self.get_logger().info("Shutting down interactive client")


def main(args=None):
    rclpy.init(args=args)
    client = InteractiveClient()

    try:
        client.run()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
