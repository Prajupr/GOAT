#!/usr/bin/env python3
"""Scenario runner for the Butler Delivery system."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
from std_srvs.srv import Empty
import sys
import time


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("butler_delivery_scenario_runner")
        self.order_publisher = self.create_publisher(Int32MultiArray, "/butler/orders", 10)
        self.cancel_publisher = self.create_publisher(Int32, "/butler/cancel_order", 10)
        self.kitchen_client = self.create_client(Empty, "/butler/confirm_at_kitchen")
        self.table_client = self.create_client(Empty, "/butler/confirm_at_table")

        self.get_logger().info("Scenario runner ready")

    def wait_for_services(self):
        self.get_logger().info("Waiting for confirm services...")
        self.kitchen_client.wait_for_service(timeout_sec=10.0)
        self.table_client.wait_for_service(timeout_sec=10.0)

    def publish_order(self, table_ids):
        msg = Int32MultiArray()
        msg.data = table_ids
        self.order_publisher.publish(msg)
        self.get_logger().info(f"Published order for tables: {table_ids}")

    def cancel_order(self, table_id):
        msg = Int32()
        msg.data = table_id
        self.cancel_publisher.publish(msg)
        self.get_logger().info(f"Published cancel for table {table_id}")

    def call_service(self, client, name):
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error(f"Service call failed: {name}")
        else:
            self.get_logger().info(f"Service called: {name}")

    def scenario_1(self):
        self.publish_order([1])
        time.sleep(1.5)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")
        time.sleep(1.5)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        self.get_logger().info("Scenario 1 complete")

    def scenario_5(self):
        self.publish_order([1, 2, 3])
        time.sleep(1.5)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")
        time.sleep(1.5)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        time.sleep(1.5)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        time.sleep(1.5)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        self.get_logger().info("Scenario 5 complete")

    def scenario_7(self):
        self.publish_order([1, 2, 3])
        time.sleep(1.0)
        self.cancel_order(2)
        time.sleep(1.0)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        time.sleep(1.5)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        self.get_logger().info("Scenario 7 complete")


def main(args=None):
    rclpy.init(args=args)
    runner = ScenarioRunner()
    runner.wait_for_services()

    if len(sys.argv) < 2:
        print("Usage: ros2 run butler_delivery scenario_runner <scenario_number>")
        print("  1 - single order with kitchen/table confirmations")
        print("  5 - multiple orders sequential delivery")
        print("  7 - multiple orders with table 2 cancellation")
        rclpy.shutdown()
        return

    scenario = sys.argv[1]
    if scenario == "1":
        runner.scenario_1()
    elif scenario == "5":
        runner.scenario_5()
    elif scenario == "7":
        runner.scenario_7()
    else:
        print(f"Unknown scenario: {scenario}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
