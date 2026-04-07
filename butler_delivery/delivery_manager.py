#!/usr/bin/env python3
"""
Butler Robot Delivery Manager — Nav2 integrated
Navigates the TurtleBot3 in Gazebo using Nav2 NavigateToPose action.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import Empty
from std_msgs.msg import String, Int32MultiArray, Int32
from visualization_msgs.msg import Marker, MarkerArray
import json
from enum import Enum
from typing import List, Dict, Optional
import time
import math


class DeliveryState(Enum):
    IDLE = "idle"
    GOING_TO_KITCHEN = "going_to_kitchen"
    WAITING_AT_KITCHEN = "waiting_at_kitchen"
    GOING_TO_TABLE = "going_to_table"
    DELIVERING_FOOD = "delivering_food"
    RETURNING_TO_KITCHEN_BEFORE_HOME = "returning_to_kitchen_before_home"
    RETURNING_HOME = "returning_home"


class TableOrder:
    def __init__(self, table_id: int, order_id: str):
        self.table_id = table_id
        self.order_id = order_id
        self.status = "pending"
        self.timestamp = time.time()
        self.confirmed_at_kitchen = False
        self.collected = False
        self.confirmed_at_table = False


class ButlerDeliveryManager(Node):
    """Delivery manager that drives TurtleBot3 via Nav2."""

    # New positions for the larger restaurant world (12m x 10m)
    # Home: near reception area
    HOME_POSITION = {"x": 0.0, "y": 0.0, "z": 0.0}
    # Kitchen: just inside the kitchen doorway (right side of partition)
    KITCHEN_POSITION = {"x": 2.8, "y": 0.0, "z": 0.0}
    # Tables in the dining area (left side)
    TABLE_POSITIONS = {
        1: {"x": -2.0, "y": 2.0, "z": 0.0},   # near table_1 (round, upper-left)
        2: {"x": -2.0, "y": -1.5, "z": 0.0},   # near table_2 (rect, lower-left)
        3: {"x": 0.0, "y": -1.5, "z": 0.0},    # near table_3 (round, center)
    }

    def __init__(self):
        super().__init__("butler_delivery_manager")
        self.callback_group = ReentrantCallbackGroup()

        self.auto_confirm = self.declare_parameter("auto_confirm", False).value
        self.auto_confirm_delay = self.declare_parameter("auto_confirm_delay", 0.5).value

        # State management
        self.current_state = DeliveryState.IDLE
        self.current_position = self.HOME_POSITION.copy()
        self.current_order_index = 0
        self.kitchen_ready = False
        self.batch_start_time = time.time()
        self.current_order_start_time = time.time()

        # Navigation state
        self._nav_goal_handle = None
        self._navigating = False
        self._nav_result_ready = False
        self._nav_succeeded = False
        self._nav_generation = 0  # incremented each navigate_to to ignore stale callbacks

        # Orders management
        self.pending_orders: List[TableOrder] = []
        self.active_orders: List[TableOrder] = []
        self.orders_in_progress: Dict[int, TableOrder] = {}

        # Timeouts
        self.kitchen_timeout = 60.0
        self.table_confirmation_timeout = 40.0
        self.check_interval = 0.2

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Publishers
        self.state_publisher = self.create_publisher(String, "/butler/state", 10)
        self.status_publisher = self.create_publisher(String, "/butler/status", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'delivery_markers', 10)
        self.marker_timer = self.create_timer(0.5, self.publish_markers)

        # Subscribers
        self.create_subscription(
            Int32MultiArray, "/butler/orders", self.orders_callback, 10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Int32, "/butler/cancel_order", self.cancel_order_topic_callback, 10,
            callback_group=self.callback_group
        )

        # Services
        self.create_service(
            Empty, "/butler/confirm_at_kitchen",
            self.confirm_at_kitchen_callback,
            callback_group=self.callback_group
        )
        self.create_service(
            Empty, "/butler/confirm_at_table",
            self.confirm_at_table_callback,
            callback_group=self.callback_group
        )
        self.create_service(
            Empty, "/butler/cancel_current_order",
            self.cancel_current_order_service_callback,
            callback_group=self.callback_group
        )

        # Main loop
        self.create_timer(
            self.check_interval, self.delivery_loop,
            callback_group=self.callback_group
        )

        self.get_logger().info("Butler Delivery Manager initialized (Nav2 mode)")
        self.get_logger().info(f"  Home:    {self.HOME_POSITION}")
        self.get_logger().info(f"  Kitchen: {self.KITCHEN_POSITION}")
        for tid, pos in self.TABLE_POSITIONS.items():
            self.get_logger().info(f"  Table {tid}: {pos}")
        self.publish_state()
        self.publish_status("Ready for orders")

    # ─── Navigation ─────────────────────────────────────────────────

    def navigate_to(self, position: Dict):
        """Send a Nav2 NavigateToPose goal."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(position["x"])
        goal.pose.pose.position.y = float(position["y"])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self._nav_generation += 1
        gen = self._nav_generation
        self._navigating = True
        self._nav_result_ready = False
        self._nav_succeeded = False

        self.get_logger().info(
            f"Navigating to ({position['x']:.1f}, {position['y']:.1f})"
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            self._navigating = False
            self._nav_result_ready = True
            self._nav_succeeded = False
            return

        send_goal_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb
        )
        send_goal_future.add_done_callback(
            lambda f, g=gen: self._nav_goal_response_cb(f, g)
        )

    def _nav_goal_response_cb(self, future, gen):
        if gen != self._nav_generation:
            return  # stale callback from a previous/canceled goal
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal rejected")
            self._navigating = False
            self._nav_result_ready = True
            self._nav_succeeded = False
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, g=gen: self._nav_result_cb(f, g)
        )

    def _nav_feedback_cb(self, feedback_msg):
        pos = feedback_msg.feedback.current_pose.pose.position
        self.current_position = {"x": pos.x, "y": pos.y, "z": pos.z}

    def _nav_result_cb(self, future, gen):
        if gen != self._nav_generation:
            return  # stale callback from a previous/canceled goal
        status = future.result().status
        # 4 = SUCCEEDED in action_msgs
        self._nav_succeeded = (status == 4)
        self._navigating = False
        self._nav_result_ready = True
        if self._nav_succeeded:
            self.get_logger().info("Navigation reached goal")
        else:
            self.get_logger().warn(f"Navigation finished with status {status}")

    def cancel_navigation(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
        self._navigating = False

    # ─── Topic / service callbacks ──────────────────────────────────

    def orders_callback(self, msg: Int32MultiArray):
        new_tables = list(msg.data)
        self.get_logger().info(f"Received orders for tables: {new_tables}")
        for table_id in new_tables:
            order = TableOrder(table_id, f"order_{table_id}_{time.time()}")
            self.pending_orders.append(order)
        self.publish_status(
            f"Pending orders: {[o.table_id for o in self.pending_orders]}"
        )

    def cancel_order_topic_callback(self, msg: Int32):
        table_id = msg.data
        if table_id == -1:
            self.cancel_current_order()
            return
        canceled = False
        for order in self.active_orders + self.pending_orders:
            if order.table_id == table_id and order.status not in [
                "canceled", "delivered", "timeout"
            ]:
                order.status = "canceled"
                canceled = True
                self.get_logger().info(f"Order for table {table_id} canceled")
                break
        if canceled:
            cur = self.get_current_order()
            if (self.current_state in [
                    DeliveryState.GOING_TO_TABLE, DeliveryState.DELIVERING_FOOD
                ] and cur and cur.table_id == table_id):
                self.cancel_navigation()
                self.advance_after_current_order()
            else:
                self.publish_status(f"Order {table_id} canceled")

    def cancel_current_order_service_callback(self, request, response):
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
        self.cancel_navigation()
        if self.current_state == DeliveryState.WAITING_AT_KITCHEN:
            self.current_state = DeliveryState.RETURNING_HOME
            self.navigate_to(self.HOME_POSITION)
            self.publish_status("Canceled at kitchen, returning home")
        elif self.current_state in [
            DeliveryState.GOING_TO_TABLE, DeliveryState.DELIVERING_FOOD
        ]:
            self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
            self.navigate_to(self.KITCHEN_POSITION)
            self.publish_status("Canceled during delivery, returning to kitchen")
        else:
            self.current_state = DeliveryState.RETURNING_HOME
            self.navigate_to(self.HOME_POSITION)
            self.publish_status("Canceled, returning home")
        self.publish_state()

    def confirm_at_kitchen_callback(self, request, response):
        if (self.current_state == DeliveryState.WAITING_AT_KITCHEN
                and self.active_orders):
            self.kitchen_ready = True
            for order in self.active_orders:
                if order.status == "waiting_kitchen":
                    order.status = "ready_for_delivery"
                    order.confirmed_at_kitchen = True
            self.get_logger().info("Kitchen confirmed food ready")
            self.publish_status("Food confirmed at kitchen")
        return response

    def confirm_at_table_callback(self, request, response):
        cur = self.get_current_order()
        if cur and self.current_state == DeliveryState.DELIVERING_FOOD:
            cur.confirmed_at_table = True
            self.get_logger().info(
                f"Customer confirmed receipt at table {cur.table_id}"
            )
            self.publish_status(f"Food confirmed at table {cur.table_id}")
        return response

    # ─── State machine ──────────────────────────────────────────────

    def delivery_loop(self):
        if self.current_state == DeliveryState.IDLE and self.pending_orders:
            self.start_next_delivery()
            return

        if self.current_state != DeliveryState.IDLE:
            self.handle_current_delivery()

        self.check_timeouts()

    def get_current_order(self) -> Optional[TableOrder]:
        if 0 <= self.current_order_index < len(self.active_orders):
            o = self.active_orders[self.current_order_index]
            if o.status not in ["canceled", "delivered", "timeout"]:
                return o
        return None

    def get_next_active_order_index(self, start_index: int) -> Optional[int]:
        for idx in range(start_index + 1, len(self.active_orders)):
            if self.active_orders[idx].status not in [
                "canceled", "delivered", "timeout"
            ]:
                return idx
        return None

    def has_active_orders(self) -> bool:
        return any(
            o.status not in ["canceled", "delivered", "timeout"]
            for o in self.active_orders
        )

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

        tables = [o.table_id for o in self.active_orders]
        self.get_logger().info(f"Starting delivery batch for tables {tables}")
        self.publish_state()
        self.publish_status(f"Going to kitchen for tables {tables}")
        self.navigate_to(self.KITCHEN_POSITION)

    def handle_current_delivery(self):
        # While navigating, wait for it to finish
        if self._navigating:
            return

        if self.current_state == DeliveryState.GOING_TO_KITCHEN:
            if self._nav_result_ready:
                self.current_state = DeliveryState.WAITING_AT_KITCHEN
                self.batch_start_time = time.time()
                self.publish_state()
                self.publish_status("Arrived at kitchen, waiting for food")
                self.get_logger().info("Arrived at kitchen")
            return

        if self.current_state == DeliveryState.WAITING_AT_KITCHEN:
            if self.kitchen_ready or (
                self.auto_confirm
                and time.time() - self.batch_start_time > self.auto_confirm_delay
            ):
                self.kitchen_ready = True
                for order in self.active_orders:
                    if order.status == "waiting_kitchen":
                        order.status = "ready_for_delivery"
                        order.confirmed_at_kitchen = True

                next_idx = self.get_next_active_order_index(-1)
                if next_idx is None:
                    self.current_state = DeliveryState.RETURNING_HOME
                    self.navigate_to(self.HOME_POSITION)
                    self.publish_state()
                    self.publish_status("No active orders, returning home")
                    return
                self.current_order_index = next_idx
                self.current_state = DeliveryState.GOING_TO_TABLE
                self.current_order_start_time = time.time()
                order = self.active_orders[next_idx]
                self.publish_state()
                self.publish_status(f"Collected food, heading to table {order.table_id}")
                self.get_logger().info(f"Heading to table {order.table_id}")
                if order.table_id in self.TABLE_POSITIONS:
                    self.navigate_to(self.TABLE_POSITIONS[order.table_id])
            return

        if self.current_state == DeliveryState.GOING_TO_TABLE:
            if self._nav_result_ready:
                cur = self.get_current_order()
                if not cur:
                    self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
                    self.navigate_to(self.KITCHEN_POSITION)
                    self.publish_state()
                    return
                cur.status = "delivering"
                self.current_state = DeliveryState.DELIVERING_FOOD
                self.current_order_start_time = time.time()
                self.publish_state()
                self.publish_status(
                    f"Arrived at table {cur.table_id}, delivering food"
                )
                self.get_logger().info(
                    f"Arrived at table {cur.table_id}, delivering"
                )
            return

        if self.current_state == DeliveryState.DELIVERING_FOOD:
            cur = self.get_current_order()
            if not cur:
                self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
                self.navigate_to(self.KITCHEN_POSITION)
                self.publish_state()
                return
            if cur.confirmed_at_table or (
                self.auto_confirm
                and time.time() - self.current_order_start_time > self.auto_confirm_delay
            ):
                cur.confirmed_at_table = True
                cur.status = "delivered"
                self.publish_status(
                    f"Food delivered to table {cur.table_id}"
                )
                self.get_logger().info(
                    f"Food delivered to table {cur.table_id}"
                )
                self.advance_after_current_order()
            return

        if self.current_state == DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME:
            if self._nav_result_ready:
                self.current_state = DeliveryState.RETURNING_HOME
                self.navigate_to(self.HOME_POSITION)
                self.publish_state()
                self.publish_status("Returned to kitchen, heading home")
                self.get_logger().info("At kitchen, heading home")
            return

        if self.current_state == DeliveryState.RETURNING_HOME:
            if self._nav_result_ready:
                self.get_logger().info("Returned to home position")
                for order in self.active_orders:
                    if order.status not in ["delivered", "canceled", "timeout"]:
                        order.status = "completed"
                for order in self.active_orders:
                    self.orders_in_progress[order.table_id] = order
                self.active_orders = []
                self.current_order_index = 0
                self.current_state = DeliveryState.IDLE
                self.publish_state()
                self.publish_status("Returned home, ready for orders")
            return

    def advance_after_current_order(self):
        next_idx = self.get_next_active_order_index(self.current_order_index)
        if next_idx is not None:
            self.current_order_index = next_idx
            self.current_state = DeliveryState.GOING_TO_TABLE
            order = self.active_orders[next_idx]
            self.publish_state()
            self.publish_status(f"Moving to next table {order.table_id}")
            if order.table_id in self.TABLE_POSITIONS:
                self.navigate_to(self.TABLE_POSITIONS[order.table_id])
            return
        self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
        self.navigate_to(self.KITCHEN_POSITION)
        self.publish_state()
        self.publish_status("No more tables, returning to kitchen")

    def check_timeouts(self):
        if self.current_state == DeliveryState.WAITING_AT_KITCHEN:
            if (not self.kitchen_ready
                    and time.time() - self.batch_start_time > self.kitchen_timeout):
                self.get_logger().warn("Timeout waiting at kitchen")
                for order in self.active_orders:
                    if order.status not in ["delivered", "canceled", "timeout"]:
                        order.status = "timeout"
                self.current_state = DeliveryState.RETURNING_HOME
                self.navigate_to(self.HOME_POSITION)
                self.publish_state()
                self.publish_status("Kitchen timeout, returning home")
                return

        if self.current_state == DeliveryState.DELIVERING_FOOD:
            cur = self.get_current_order()
            if not cur:
                return
            if (not cur.confirmed_at_table
                    and time.time() - self.current_order_start_time
                    > self.table_confirmation_timeout):
                self.get_logger().warn(f"Timeout at table {cur.table_id}")
                cur.status = "timeout"
                next_idx = self.get_next_active_order_index(self.current_order_index)
                if next_idx is not None:
                    self.current_order_index = next_idx
                    self.current_state = DeliveryState.GOING_TO_TABLE
                    order = self.active_orders[next_idx]
                    self.publish_state()
                    self.publish_status(
                        f"Table {cur.table_id} timed out, going to {order.table_id}"
                    )
                    if order.table_id in self.TABLE_POSITIONS:
                        self.navigate_to(self.TABLE_POSITIONS[order.table_id])
                else:
                    self.current_state = DeliveryState.RETURNING_TO_KITCHEN_BEFORE_HOME
                    self.navigate_to(self.KITCHEN_POSITION)
                    self.publish_state()
                    self.publish_status("No remaining tables, heading home")

    # ─── Publishing ─────────────────────────────────────────────────

    def publish_state(self):
        msg = String()
        cur = self.get_current_order()
        msg.data = json.dumps({
            "state": self.current_state.value,
            "current_table": cur.table_id if cur else None,
            "position": self.current_position,
            "pending_orders": [o.table_id for o in self.pending_orders],
            "active_orders": [
                {"table": o.table_id, "status": o.status}
                for o in self.active_orders
            ],
        })
        self.state_publisher.publish(msg)

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def publish_markers(self):
        ma = MarkerArray()
        mid = 0

        # Robot position
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "robot"
        m.id = mid; mid += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.current_position["x"]
        m.pose.position.y = self.current_position["y"]
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.g = 1.0; m.color.a = 1.0
        ma.markers.append(m)

        # State label
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "state"
        m.id = mid; mid += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = self.current_position["x"]
        m.pose.position.y = self.current_position["y"] + 0.5
        m.pose.position.z = 0.8
        m.pose.orientation.w = 1.0
        m.scale.z = 0.2
        m.color.r = m.color.g = m.color.b = 1.0; m.color.a = 1.0
        m.text = f"State: {self.current_state.value}"
        ma.markers.append(m)

        # Location markers
        locations = [
            ("Kitchen", self.KITCHEN_POSITION, (1.0, 0.0, 0.0)),
            ("Home", self.HOME_POSITION, (0.0, 0.0, 1.0)),
        ]
        for tid, pos in self.TABLE_POSITIONS.items():
            locations.append((f"Table {tid}", pos, (0.5, 0.5, 0.5)))

        for name, pos, color in locations:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "locations"
            m.id = mid; mid += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = pos["x"]
            m.pose.position.y = pos["y"]
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.4
            m.scale.z = 0.05
            m.color.r = color[0]; m.color.g = color[1]; m.color.b = color[2]
            m.color.a = 0.7
            ma.markers.append(m)

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "labels"
            m.id = mid; mid += 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = pos["x"]
            m.pose.position.y = pos["y"] + 0.3
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.z = 0.15
            m.color.r = m.color.g = m.color.b = 1.0; m.color.a = 1.0
            m.text = name
            ma.markers.append(m)

        self.marker_publisher.publish(ma)


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
