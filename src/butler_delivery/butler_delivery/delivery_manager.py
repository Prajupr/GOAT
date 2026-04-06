#!/usr/bin/env python3
"""
Butler Robot Delivery Manager
Handles multi-table delivery with timeout, cancellation, and complex task scenarios
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from std_msgs.msg import String, Int32MultiArray, Int32
import json
from enum import Enum
from typing import List, Dict, Optional
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class DeliveryState(Enum):
    """States for delivery state machine"""
    IDLE = "idle"
    GOING_TO_KITCHEN = "going_to_kitchen"
    WAITING_AT_KITCHEN = "waiting_at_kitchen"
    GOING_TO_TABLE = "going_to_table"
    DELIVERING_FOOD = "delivering_food"
    RETURNING_TO_KITCHEN_BEFORE_HOME = "returning_to_kitchen_before_home"
    RETURNING_HOME = "returning_home"


class TableOrder:
    """Represents an order for a specific table"""
    def __init__(self, table_id: int, order_id: str):
        self.table_id = table_id
        self.order_id = order_id
        self.status = "pending"  # pending, confirmed, delivered, timeout, canceled
        self.timestamp = time.time()
        self.confirmed_at_kitchen = False
        self.collected = False
        self.confirmed_at_table = False


class ButlerDeliveryManager(Node):
    """
    Main delivery manager for butler robot
    Handles all delivery scenarios from the assessment
    """

    # Home position (0, 0)
    HOME_POSITION = {"x": 0.0, "y": 0.0, "z": 0.0}

    # Kitchen position
    KITCHEN_POSITION = {"x": 1.0, "y": 0.0, "z": 0.0}

    # Table positions
    TABLE_POSITIONS = {
        1: {"x": -0.5, "y": 0.5, "z": 0.0},
        2: {"x": -0.5, "y": -0.5, "z": 0.0},
        3: {"x": 1.5, "y": -0.5, "z": 0.0},
    }

    def __init__(self):
        super().__init__("butler_delivery_manager")
        self.callback_group = ReentrantCallbackGroup()

        # Runtime parameters
        self.auto_confirm = self.declare_parameter("auto_confirm", False).value
        self.auto_confirm_delay = self.declare_parameter("auto_confirm_delay", 0.5).value

        # State management
        self.current_state = DeliveryState.IDLE
        self.current_position = self.HOME_POSITION.copy()
        self.current_order_index = 0
        self.kitchen_ready = False
        self.batch_start_time = time.time()
        self.current_order_start_time = time.time()

        # Orders management
        self.pending_orders: List[TableOrder] = []
        self.active_orders: List[TableOrder] = []
        self.orders_in_progress: Dict[int, TableOrder] = {}

        # Configuration
        self.kitchen_timeout = 30.0  # seconds
        self.table_confirmation_timeout = 20.0  # seconds
        self.check_interval = 0.1  # seconds

        # Publishers
        self.state_publisher = self.create_publisher(
            String, "/butler/state", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/butler/status", 10
        )
        self.marker_publisher = self.create_publisher(MarkerArray, 'delivery_markers', 10)
        
        # Timer for marker updates
        self.marker_timer = self.create_timer(0.5, self.publish_markers)

        # Subscribers
        self.create_subscription(
            Int32MultiArray,
            "/butler/orders",
            self.orders_callback,
            10,
            callback_group=self.callback_group
        )

        self.create_subscription(
            Int32,
            "/butler/cancel_order",
            self.cancel_order_topic_callback,
            10,
            callback_group=self.callback_group
        )

        # Services
        self.create_service(
            Empty,
            "/butler/confirm_at_kitchen",
            self.confirm_at_kitchen_callback,
            callback_group=self.callback_group
        )

        self.create_service(
            Empty,
            "/butler/confirm_at_table",
            self.confirm_at_table_callback,
            callback_group=self.callback_group
        )

        self.create_service(
            Empty,
            "/butler/cancel_current_order",
            self.cancel_current_order_service_callback,
            callback_group=self.callback_group
        )

        # Main delivery loop
        self.create_timer(
            self.check_interval,
            self.delivery_loop,
            callback_group=self.callback_group
        )

        self.get_logger().info("Butler Delivery Manager initialized")
        self.publish_state()
        self.publish_status("Ready for orders")

    def orders_callback(self, msg: Int32MultiArray):
        """
        Handle incoming orders
        msg.data: array of table IDs for new orders
        """
        new_tables = list(msg.data)
        self.get_logger().info(f"Received orders for tables: {new_tables}")

        for table_id in new_tables:
            order = TableOrder(table_id, f"order_{table_id}_{time.time()}")
            self.pending_orders.append(order)

        self.publish_status(f"Pending orders: {[o.table_id for o in self.pending_orders]}")

    def cancel_order_topic_callback(self, msg: Int32):
        """Cancel a specific table or current order via topic."""
        table_id = msg.data
        if table_id == -1:
            self.cancel_current_order()
            return

        canceled = False
        for order in self.active_orders + self.pending_orders:
            if order.table_id == table_id and order.status not in ["canceled", "delivered", "timeout"]:
                order.status = "canceled"
                order.timeout_reason = "user_cancel"
                canceled = True
                self.get_logger().info(f"Order for table {table_id} canceled by request")
                break

        if canceled:
            if self.current_state in [DeliveryState.GOING_TO_TABLE, DeliveryState.DELIVERING_FOOD] and self.get_current_order() and self.get_current_order().table_id == table_id:
                self.get_logger().info(f"Current active table {table_id} is canceled, moving to next table or kitchen")
                self.advance_after_current_order()
            else:
                self.publish_status(f"Order {table_id} canceled")
        else:
            self.get_logger().warn(f"Cancel request for table {table_id} did not match any active or pending order")

    def cancel_current_order_service_callback(self, request, response):
        """Service: Cancel the current active batch or current table."""
        self.cancel_current_order()
        return response

    def cancel_current_order(self):
        if not self.active_orders:
            self.get_logger().warn("No active orders to cancel")
            return

        self.get_logger().info("Cancelling current active delivery batch")
        for order in self.active_orders:
            if order.status not in ["delivered", "canceled", "timeout"]:
                order.status = "canceled"
                order.timeout_reason = "user_cancel"

        if self.current_state == DeliveryState.WAITING_AT_KITCHEN:
            self.current_state = DeliveryState.RETURNING_HOME
            self.publish_status("Canceled at kitchen, returning home")
        elif self.current_state in [DeliveryState.GOING_TO_TABLE, DeliveryState.DELIVERING_FOOD]:
            self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
            self.publish_status("Canceled during delivery, returning to kitchen first")
        else:
            self.current_state = DeliveryState.RETURNING_HOME
            self.publish_status("Canceled, returning home")

        self.publish_state()

    def delivery_loop(self):
        """Main delivery state machine loop"""
        if self.current_state == DeliveryState.IDLE and self.pending_orders:
            self.start_next_delivery()

        if self.current_state != DeliveryState.IDLE:
            self.handle_current_delivery()

        self.check_timeouts()

    def get_current_order(self) -> Optional[TableOrder]:
        if 0 <= self.current_order_index < len(self.active_orders):
            order = self.active_orders[self.current_order_index]
            if order.status not in ["canceled", "delivered", "timeout"]:
                return order
        return None

    def get_next_active_order_index(self, start_index: int) -> Optional[int]:
        for idx in range(start_index + 1, len(self.active_orders)):
            if self.active_orders[idx].status not in ["canceled", "delivered", "timeout"]:
                return idx
        return None

    def has_active_orders(self) -> bool:
        return any(order.status not in ["canceled", "delivered", "timeout"] for order in self.active_orders)

    def handle_current_delivery(self):
        """Handle states for current delivery"""
        if self.current_state == DeliveryState.GOING_TO_KITCHEN:
            self.move_to_position(self.KITCHEN_POSITION)
            self.current_state = DeliveryState.WAITING_AT_KITCHEN
            self.batch_start_time = time.time()
            self.publish_state()
            self.publish_status("Arrived at kitchen, waiting for food")
            self.get_logger().info("Arrived at kitchen, waiting for food")
            return

        if self.current_state == DeliveryState.WAITING_AT_KITCHEN:
            if self.kitchen_ready or (self.auto_confirm and time.time() - self.batch_start_time > self.auto_confirm_delay):
                self.kitchen_ready = True
                for order in self.active_orders:
                    if order.status == "waiting_kitchen":
                        order.status = "ready_for_delivery"
                        order.confirmed_at_kitchen = True

                next_index = self.get_next_active_order_index(-1)
                if next_index is None:
                    self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
                    self.publish_state()
                    self.publish_status("No active orders after kitchen, returning home")
                    self.get_logger().info("No active orders after kitchen")
                    return

                self.current_order_index = next_index
                self.current_state = DeliveryState.GOING_TO_TABLE
                self.current_order_start_time = time.time()
                self.publish_state()
                self.publish_status(f"Collected food for tables {[o.table_id for o in self.active_orders if o.status != 'canceled']}")
                self.get_logger().info("Collected food from kitchen")
            return

        if self.current_state == DeliveryState.GOING_TO_TABLE:
            current_order = self.get_current_order()
            if not current_order:
                self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
                self.publish_state()
                return

            table_id = current_order.table_id
            if table_id in self.TABLE_POSITIONS:
                self.move_to_position(self.TABLE_POSITIONS[table_id])
            current_order.status = "delivering"
            self.current_state = DeliveryState.DELIVERING_FOOD
            self.current_order_start_time = time.time()
            self.publish_state()
            self.publish_status(f"Arrived at table {table_id}, delivering food")
            self.get_logger().info(f"Arrived at table {table_id}, delivering food")
            return

        if self.current_state == DeliveryState.DELIVERING_FOOD:
            current_order = self.get_current_order()
            if not current_order:
                self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
                self.publish_state()
                return
            if current_order.confirmed_at_table or (self.auto_confirm and time.time() - self.current_order_start_time > self.auto_confirm_delay):
                current_order.confirmed_at_table = True
                current_order.status = "delivered"
                self.publish_status(f"Food delivered to table {current_order.table_id}")
                self.get_logger().info(f"Food delivered to table {current_order.table_id}")
                self.advance_after_current_order()
            return

        if self.current_state == DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME:
            self.move_to_position(self.KITCHEN_POSITION)
            self.current_state = DeliveryState.RETURNING_HOME
            self.publish_state()
            self.publish_status("Returned to kitchen before home")
            self.get_logger().info("Returned to kitchen before home")
            return

        if self.current_state == DeliveryState.RETURNING_HOME:
            self.move_to_position(self.HOME_POSITION)
            self.publish_state()
            self.publish_status("Returned home and ready for next orders")
            self.get_logger().info("Returned to home position")
            for order in self.active_orders:
                if order.status not in ["delivered", "canceled", "timeout"]:
                    order.status = "completed"
            for order in self.active_orders:
                self.orders_in_progress[order.table_id] = order
            self.active_orders = []
            self.current_order_index = 0
            self.current_state = DeliveryState.IDLE
            return

    def advance_after_current_order(self):
        next_index = self.get_next_active_order_index(self.current_order_index)
        if next_index is not None:
            self.current_order_index = next_index
            self.current_state = DeliveryState.GOING_TO_TABLE
            self.publish_state()
            self.publish_status(f"Moving to next table {self.active_orders[next_index].table_id}")
            return

        self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
        self.publish_state()
        self.publish_status("No more tables, returning to kitchen before home")

    def check_timeouts(self):
        """Check for timeouts in various states"""
        if self.current_state == DeliveryState.WAITING_AT_KITCHEN:
            elapsed = time.time() - self.batch_start_time
            if not self.kitchen_ready and elapsed > self.kitchen_timeout:
                self.get_logger().warn("Timeout waiting at kitchen")
                self.handle_kitchen_timeout()
                return

        if self.current_state == DeliveryState.DELIVERING_FOOD:
            current_order = self.get_current_order()
            if not current_order:
                return
            elapsed = time.time() - self.current_order_start_time
            if not current_order.confirmed_at_table and elapsed > self.table_confirmation_timeout:
                self.get_logger().warn(f"Timeout waiting at table {current_order.table_id}")
                self.handle_table_timeout()

    def handle_kitchen_timeout(self):
        for order in self.active_orders:
            if order.status not in ["delivered", "canceled", "timeout"]:
                order.status = "timeout"
                order.timeout_reason = "kitchen_timeout"
        self.current_state = DeliveryState.RETURNING_HOME
        self.publish_state()
        self.publish_status("Kitchen timeout, returning home")

    def handle_table_timeout(self):
        current_order = self.get_current_order()
        if not current_order:
            return
        current_order.status = "timeout"
        current_order.timeout_reason = "table_timeout"
        self.get_logger().info(f"Table {current_order.table_id} timed out, skipping to next")
        next_index = self.get_next_active_order_index(self.current_order_index)
        if next_index is not None:
            self.current_order_index = next_index
            self.current_state = DeliveryState.GOING_TO_TABLE
            self.publish_state()
            self.publish_status(f"Skipping table {current_order.table_id}, moving to table {self.active_orders[next_index].table_id}")
        else:
            self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
            self.publish_state()
            self.publish_status(f"No remaining tables, returning to kitchen before home")

    def start_next_delivery(self):
        if not self.pending_orders:
            return
        self.active_orders = self.pending_orders.copy()
        self.pending_orders = []
        for order in self.active_orders:
            order.status = "waiting_kitchen"
            order.timestamp = time.time()

        self.current_order_index = 0
        self.kitchen_ready = False
        self.current_state = DeliveryState.GOING_TO_KITCHEN
        self.batch_start_time = time.time()

        self.get_logger().info(f"Starting delivery batch for tables {[o.table_id for o in self.active_orders]}")
        self.publish_state()
        self.publish_status(f"Going to kitchen for tables {[o.table_id for o in self.active_orders]}")

    def confirm_at_kitchen_callback(self, request, response):
        if self.current_state == DeliveryState.WAITING_AT_KITCHEN and self.active_orders:
            self.kitchen_ready = True
            for order in self.active_orders:
                if order.status == "waiting_kitchen":
                    order.status = "ready_for_delivery"
                    order.confirmed_at_kitchen = True
            self.get_logger().info(f"Kitchen confirmed food ready for tables {[o.table_id for o in self.active_orders]}")
            self.publish_status(f"Food confirmed at kitchen for tables {[o.table_id for o in self.active_orders]}")
        return response

    def confirm_at_table_callback(self, request, response):
        current_order = self.get_current_order()
        if current_order and self.current_state == DeliveryState.DELIVERING_FOOD:
            current_order.confirmed_at_table = True
            self.get_logger().info(f"Customer confirmed food receipt at table {current_order.table_id}")
            self.publish_status(f"Food confirmed at table {current_order.table_id}")
        return response

    def move_to_position(self, position: Dict):
        self.current_position = position.copy()

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({
            "state": self.current_state.value,
            "current_table": self.get_current_order().table_id if self.get_current_order() else None,
            "position": self.current_position,
            "pending_orders": [o.table_id for o in self.pending_orders],
            "active_orders": [{"table": o.table_id, "status": o.status} for o in self.active_orders],
        })
        self.state_publisher.publish(msg)

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def publish_markers(self):
        """Publish RViz markers for visualization"""
        marker_array = MarkerArray()
        marker_id = 0

        # Robot position marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "world"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = marker_id
        marker_id += 1
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.current_position["x"]
        robot_marker.pose.position.y = self.current_position["y"]
        robot_marker.pose.position.z = self.current_position["z"]
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.2
        robot_marker.scale.y = 0.2
        robot_marker.scale.z = 0.2
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0
        marker_array.markers.append(robot_marker)

        # State text marker
        state_marker = Marker()
        state_marker.header.frame_id = "world"
        state_marker.header.stamp = self.get_clock().now().to_msg()
        state_marker.ns = "state"
        state_marker.id = marker_id
        marker_id += 1
        state_marker.type = Marker.TEXT_VIEW_FACING
        state_marker.action = Marker.ADD
        state_marker.pose.position.x = self.current_position["x"]
        state_marker.pose.position.y = self.current_position["y"] + 0.3
        state_marker.pose.position.z = 0.5
        state_marker.pose.orientation.w = 1.0
        state_marker.scale.z = 0.1
        state_marker.color.r = 1.0
        state_marker.color.g = 1.0
        state_marker.color.b = 1.0
        state_marker.color.a = 1.0
        state_marker.text = f"State: {self.current_state.value}"
        marker_array.markers.append(state_marker)

        # Current order text marker
        current_order = self.get_current_order()
        if current_order:
            order_marker = Marker()
            order_marker.header.frame_id = "world"
            order_marker.header.stamp = self.get_clock().now().to_msg()
            order_marker.ns = "order"
            order_marker.id = marker_id
            marker_id += 1
            order_marker.type = Marker.TEXT_VIEW_FACING
            order_marker.action = Marker.ADD
            order_marker.pose.position.x = self.current_position["x"]
            order_marker.pose.position.y = self.current_position["y"] + 0.5
            order_marker.pose.position.z = 0.5
            order_marker.pose.orientation.w = 1.0
            order_marker.scale.z = 0.08
            order_marker.color.r = 1.0
            order_marker.color.g = 0.5
            order_marker.color.b = 0.0
            order_marker.color.a = 1.0
            order_marker.text = f"Table {current_order.table_id}: {current_order.status}"
            marker_array.markers.append(order_marker)

        # Location markers (kitchen and tables)
        locations = [
            ("kitchen", self.KITCHEN_POSITION, (1.0, 0.0, 0.0)),
            ("home", self.HOME_POSITION, (0.0, 0.0, 1.0)),
        ]
        for table_id, pos in self.TABLE_POSITIONS.items():
            locations.append((f"table_{table_id}", pos, (0.5, 0.5, 0.5)))

        for name, pos, color in locations:
            loc_marker = Marker()
            loc_marker.header.frame_id = "world"
            loc_marker.header.stamp = self.get_clock().now().to_msg()
            loc_marker.ns = "locations"
            loc_marker.id = marker_id
            marker_id += 1
            loc_marker.type = Marker.CUBE
            loc_marker.action = Marker.ADD
            loc_marker.pose.position.x = pos["x"]
            loc_marker.pose.position.y = pos["y"]
            loc_marker.pose.position.z = 0.0
            loc_marker.pose.orientation.w = 1.0
            loc_marker.scale.x = 0.3
            loc_marker.scale.y = 0.3
            loc_marker.scale.z = 0.05
            loc_marker.color.r = color[0]
            loc_marker.color.g = color[1]
            loc_marker.color.b = color[2]
            loc_marker.color.a = 0.7
            marker_array.markers.append(loc_marker)

            # Location label
            label_marker = Marker()
            label_marker.header.frame_id = "world"
            label_marker.header.stamp = self.get_clock().now().to_msg()
            label_marker.ns = "labels"
            label_marker.id = marker_id
            marker_id += 1
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position.x = pos["x"]
            label_marker.pose.position.y = pos["y"] + 0.2
            label_marker.pose.position.z = 0.1
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = 0.05
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 1.0
            label_marker.text = name.replace("_", " ").title()
            marker_array.markers.append(label_marker)

        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    manager = ButlerDeliveryManager()
    executor = MultiThreadedExecutor()
    executor.add_node(manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        manager.get_logger().info("Shutting down delivery manager")
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
