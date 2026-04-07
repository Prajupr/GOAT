#!/usr/bin/env python3
"""
Example order publisher for testing the butler delivery system
Publishes orders for different scenarios
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys
from typing import List


class OrderPublisher(Node):
    """Publishes test orders to the delivery manager"""
    
    def __init__(self):
        super().__init__("order_publisher")
        self.publisher = self.create_publisher(
            Int32MultiArray,
            "/butler/orders",
            10
        )
        self.get_logger().info("Order Publisher ready")
    
    def publish_order(self, table_ids: List[int]):
        """Publish an order for the specified tables"""
        msg = Int32MultiArray()
        msg.data = table_ids
        self.publisher.publish(msg)
        self.get_logger().info(f"Published order for tables: {table_ids}")
    
    def scenario_1(self):
        """Scenario 1: Single order to table 1"""
        self.get_logger().info("=== Scenario 1: Single order to table 1 ===")
        self.publish_order([1])
    
    def scenario_2(self):
        """Scenario 2: Single order with timeout at kitchen"""
        self.get_logger().info("=== Scenario 2: Order with kitchen timeout ===")
        self.publish_order([2])
    
    def scenario_3(self):
        """Scenario 3: Delivery to table with confirmation timeout"""
        self.get_logger().info("=== Scenario 3: Order with table confirmation timeout ===")
        self.publish_order([3])
    
    def scenario_5(self):
        """Scenario 5: Multiple orders in sequence"""
        self.get_logger().info("=== Scenario 5: Multiple orders (table1, table2, table3) ===")
        self.publish_order([1, 2, 3])
    
    def scenario_6(self):
        """Scenario 6: Multiple orders with one timeout"""
        self.get_logger().info("=== Scenario 6: Orders 1,2,3 with handling ===")
        self.publish_order([1])
        rclpy.spin_once(self, timeout_sec=0.5)
        self.publish_order([2])
        rclpy.spin_once(self, timeout_sec=0.5)
        self.publish_order([3])


def main(args=None):
    rclpy.init(args=args)
    
    publisher = OrderPublisher()
    
    # Show usage
    if len(sys.argv) < 2:
        print("""
Usage: ros2 run butler_delivery order_publisher <scenario>

Scenarios:
  1 - Single order to table 1
  2 - Single order with kitchen timeout
  3 - Single order with table confirmation timeout
  5 - Multiple orders (tables 1, 2, 3)
  6 - Multiple orders with handling
        """)
        rclpy.shutdown()
        return
    
    scenario = sys.argv[1]
    
    if scenario == "1":
        publisher.scenario_1()
    elif scenario == "2":
        publisher.scenario_2()
    elif scenario == "3":
        publisher.scenario_3()
    elif scenario == "5":
        publisher.scenario_5()
    elif scenario == "6":
        publisher.scenario_6()
    else:
        print(f"Unknown scenario: {scenario}")
    
    # Keep the node running briefly for publishing
    for i in range(3):
        rclpy.spin_once(publisher, timeout_sec=0.1)
    
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
