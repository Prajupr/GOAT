#!/usr/bin/env python3
"""Scenario runner for the Butler Delivery system.

Subscribes to /butler/state and waits for the robot to reach the expected
state before sending confirmations. Works with real Nav2 navigation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32, String
from std_srvs.srv import Empty
import sys
import time
import json


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("butler_delivery_scenario_runner")
        self.order_publisher = self.create_publisher(Int32MultiArray, "/butler/orders", 10)
        self.cancel_publisher = self.create_publisher(Int32, "/butler/cancel_order", 10)
        self.kitchen_client = self.create_client(Empty, "/butler/confirm_at_kitchen")
        self.table_client = self.create_client(Empty, "/butler/confirm_at_table")

        self.current_state = "idle"
        self.current_table = None
        self.status_text = ""

        self.create_subscription(String, "/butler/state", self._state_cb, 10)
        self.create_subscription(String, "/butler/status", self._status_cb, 10)

        self.get_logger().info("Scenario runner ready")

    def _state_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.current_state = data.get("state", self.current_state)
            self.current_table = data.get("current_table", self.current_table)
        except json.JSONDecodeError:
            pass

    def _status_cb(self, msg: String):
        self.status_text = msg.data
        self.get_logger().info(f"[butler] {msg.data}")

    def wait_for_services(self):
        self.get_logger().info("Waiting for confirm services...")
        self.kitchen_client.wait_for_service(timeout_sec=30.0)
        self.table_client.wait_for_service(timeout_sec=30.0)
        # Allow publisher-subscriber matching to settle
        time.sleep(2.0)
        self.get_logger().info("Services ready")

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            self.get_logger().error(f"Service call failed: {name}")
        else:
            self.get_logger().info(f"Service called: {name}")

    def wait_for_state(self, target_state: str, timeout: float = 120.0):
        """Spin until butler reaches the target state or timeout."""
        self.get_logger().info(f"Waiting for state '{target_state}' ...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.current_state == target_state:
                self.get_logger().info(f"State reached: {target_state}")
                return True
        self.get_logger().warn(
            f"Timeout ({timeout}s) waiting for state '{target_state}', "
            f"current: '{self.current_state}'"
        )
        return False

    def wait_for_state_in(self, target_states: list, timeout: float = 120.0):
        """Spin until butler reaches one of the target states."""
        self.get_logger().info(f"Waiting for state in {target_states} ...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.current_state in target_states:
                self.get_logger().info(f"State reached: {self.current_state}")
                return True
        self.get_logger().warn(
            f"Timeout ({timeout}s) waiting for states {target_states}, "
            f"current: '{self.current_state}'"
        )
        return False

    def wait_for_idle(self, timeout: float = 180.0):
        """Wait until robot returns to idle."""
        return self.wait_for_state("idle", timeout)

    # ─── Scenarios ──────────────────────────────────────────────────

    def scenario_1(self):
        """Single table delivery."""
        self.get_logger().info("=== SCENARIO 1: Single table order ===")
        self.publish_order([1])

        # Wait for robot to arrive at kitchen
        if not self.wait_for_state("waiting_at_kitchen"):
            return
        time.sleep(1.0)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")

        # Wait for robot to arrive at table (delivering_food)
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")

        # Wait for return home
        self.wait_for_idle()
        self.get_logger().info("=== SCENARIO 1 COMPLETE ===")

    def scenario_2(self):
        """Table confirmation timeout — customer never confirms receipt."""
        self.get_logger().info("=== SCENARIO 2: Table confirmation timeout ===")
        self.publish_order([1])

        # Wait for robot to arrive at kitchen
        if not self.wait_for_state("waiting_at_kitchen"):
            return
        time.sleep(1.0)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")

        # Wait for robot to arrive at table
        if not self.wait_for_state("delivering_food"):
            return
        self.get_logger().info("Robot at table — NOT confirming (waiting for timeout)...")

        # Do NOT call confirm_at_table — let the table timeout trigger
        # The delivery manager has a 40s table_confirmation_timeout
        # Robot should timeout and return to kitchen then home
        self.wait_for_idle(timeout=180.0)
        self.get_logger().info("=== SCENARIO 2 COMPLETE ===")

    def scenario_3(self):
        """Scenario 3 — Two conditions:
        (a) Kitchen never confirms → timeout → return home
        (b) Kitchen confirms, robot delivers to table, no one confirms at table
            → robot goes to kitchen first, then home
        """
        # ── Part A: Kitchen timeout ──
        self.get_logger().info("=== SCENARIO 3a: Kitchen confirmation timeout ===")
        self.publish_order([2])

        if not self.wait_for_state("waiting_at_kitchen"):
            return
        self.get_logger().info("Robot at kitchen — NOT confirming (waiting for timeout)...")

        # Do NOT call confirm_at_kitchen — 60s kitchen_timeout triggers
        # Robot should timeout and return home directly
        self.wait_for_idle(timeout=180.0)
        self.get_logger().info("=== SCENARIO 3a COMPLETE ===")

        # ── Part B: Kitchen confirms, table does NOT confirm ──
        self.get_logger().info("=== SCENARIO 3b: Table no-confirm → kitchen → home ===")
        self.publish_order([1])

        if not self.wait_for_state("waiting_at_kitchen"):
            return
        time.sleep(1.0)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")

        # Wait for robot to arrive at table
        if not self.wait_for_state("delivering_food"):
            return
        self.get_logger().info("Robot at table — NOT confirming (waiting for timeout)...")

        # Do NOT call confirm_at_table — 40s table timeout triggers
        # Robot should go to kitchen first, then home
        self.wait_for_idle(timeout=180.0)
        self.get_logger().info("=== SCENARIO 3b COMPLETE ===")

    def scenario_4(self):
        """Cancel order while robot is heading to kitchen."""
        self.get_logger().info("=== SCENARIO 4: Cancel while going to kitchen ===")
        self.publish_order([3])

        # Wait for robot to start heading to kitchen
        if not self.wait_for_state("going_to_kitchen"):
            return
        time.sleep(2.0)
        self.get_logger().info("Cancelling all orders while en route to kitchen...")
        self.cancel_order(-1)

        # Robot should abort and return home
        self.wait_for_idle(timeout=180.0)
        self.get_logger().info("=== SCENARIO 4 COMPLETE ===")

    def scenario_5(self):
        """Multi-table sequential delivery."""
        self.get_logger().info("=== SCENARIO 5: Multi-table order [1,2,3] ===")
        self.publish_order([1, 2, 3])

        # Wait for kitchen
        if not self.wait_for_state("waiting_at_kitchen"):
            return
        time.sleep(1.0)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")

        # Deliver to table 1
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")

        # Wait for transition to going_to_table (heading to table 2)
        if not self.wait_for_state("going_to_table", timeout=30.0):
            return

        # Deliver to table 2
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")

        # Wait for transition to going_to_table (heading to table 3)
        if not self.wait_for_state("going_to_table", timeout=30.0):
            return

        # Deliver to table 3
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")

        self.wait_for_idle()
        self.get_logger().info("=== SCENARIO 5 COMPLETE ===")

    def scenario_6(self):
        """Multi-table [1,2,3]: table 1 no-confirm (timeout), deliver to tables 2 & 3,
        then kitchen before home."""
        self.get_logger().info("=== SCENARIO 6: Multi-table with table 1 timeout ===")
        self.publish_order([1, 2, 3])

        # Wait for kitchen
        if not self.wait_for_state("waiting_at_kitchen"):
            return
        time.sleep(1.0)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")

        # Robot heads to table 1 — do NOT confirm (let timeout happen)
        if not self.wait_for_state("delivering_food"):
            return
        self.get_logger().info("Robot at table 1 — NOT confirming (waiting for timeout)...")

        # Wait for timeout to trigger state change away from delivering_food
        # Timeout causes state → going_to_table (heading to table 2)
        if not self.wait_for_state("going_to_table", timeout=120.0):
            return
        self.get_logger().info("Table 1 timed out — robot heading to table 2")

        # Wait for arrival at table 2
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        self.get_logger().info("Table 2 confirmed")

        # Wait for state to leave delivering_food (going_to_table for table 3)
        if not self.wait_for_state("going_to_table", timeout=30.0):
            return

        # Wait for arrival at table 3
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")
        self.get_logger().info("Table 3 confirmed")

        # After final table, robot goes to kitchen before home
        self.wait_for_idle(timeout=180.0)
        self.get_logger().info("=== SCENARIO 6 COMPLETE ===")

    def scenario_7(self):
        """Multi-table with table 2 cancellation."""
        self.get_logger().info("=== SCENARIO 7: Multi-table [1,2,3] with table 2 cancel ===")
        self.publish_order([1, 2, 3])

        # Wait for robot to be heading to kitchen or waiting
        if not self.wait_for_state_in(["going_to_kitchen", "waiting_at_kitchen"]):
            return
        time.sleep(1.0)
        self.cancel_order(2)
        self.get_logger().info("Cancelled table 2")

        # Wait for kitchen
        if not self.wait_for_state("waiting_at_kitchen"):
            return
        time.sleep(1.0)
        self.call_service(self.kitchen_client, "/butler/confirm_at_kitchen")

        # Deliver to table 1
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")

        # Wait for transition to going_to_table (heading to table 3, table 2 was cancelled)
        if not self.wait_for_state("going_to_table", timeout=30.0):
            return

        # Deliver to table 3 (table 2 was cancelled)
        if not self.wait_for_state("delivering_food"):
            return
        time.sleep(1.0)
        self.call_service(self.table_client, "/butler/confirm_at_table")

        self.wait_for_idle()
        self.get_logger().info("=== SCENARIO 7 COMPLETE ===")


def main(args=None):
    rclpy.init(args=args)
    runner = ScenarioRunner()
    runner.wait_for_services()

    if len(sys.argv) < 2:
        print("Usage: ros2 run butler_delivery scenario_runner <scenario_number>")
        print("  1 - Single table order with confirmations")
        print("  2 - Table confirmation timeout (customer never confirms)")
        print("  3 - (a) Kitchen timeout + (b) Table no-confirm → kitchen → home")
        print("  4 - Cancel order while robot is heading to kitchen")
        print("  5 - Multi-table sequential delivery [1,2,3]")
        print("  6 - Multi-table [1,2,3] with table 1 timeout, deliver 2 & 3")
        print("  7 - Multi-table with table 2 cancellation")
        rclpy.shutdown()
        return

    scenario = sys.argv[1]
    if scenario == "1":
        runner.scenario_1()
    elif scenario == "2":
        runner.scenario_2()
    elif scenario == "3":
        runner.scenario_3()
    elif scenario == "4":
        runner.scenario_4()
    elif scenario == "5":
        runner.scenario_5()
    elif scenario == "6":
        runner.scenario_6()
    elif scenario == "7":
        runner.scenario_7()
    else:
        print(f"Unknown scenario: {scenario}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
