# Butler Robot Delivery System

A ROS2 Humble package for a TurtleBot3 Waffle butler robot that autonomously delivers food from a kitchen to customer tables in a simulated restaurant environment using Nav2 navigation.

## Overview

The system uses a state-machine-based delivery manager integrated with Nav2's `NavigateToPose` action for autonomous navigation. The robot operates in a realistic 12 × 10 m Gazebo Classic restaurant world with a kitchen area, three customer tables, a bar, and a reception desk.

## Restaurant Layout

```
+----------------------------------------------------+
|                                                      |
|   Table 1 (round)            Kitchen                 |
|   (-2.0, 2.0)           |   (2.8, 0.0)              |
|                          |                           |
|             HOME (0, 0)  |   Bar                     |
|                          |                           |
|   Table 2 (rect)   Table 3 (round)   Reception      |
|  (-2.0, -1.5)     (0.0, -1.5)                       |
|                                                      |
+----------------------------------------------------+
```

## Delivery Scenarios

### Scenario 1 — Single Table Delivery
Order for one table → kitchen (confirm) → table (confirm) → home.

### Scenario 2 — Table Confirmation Timeout
Order for one table → kitchen (confirm) → table → **customer never confirms** → 40 s timeout → kitchen → home.

### Scenario 3 — Timeout Conditions (Two Parts)
**Part A — Kitchen timeout:** Order → kitchen → **kitchen never confirms** → 60 s timeout → home.
**Part B — Table timeout (single order):** Order → kitchen (confirm) → table → **customer never confirms** → 40 s timeout → kitchen → home.

### Scenario 4 — Cancel While Heading to Kitchen
Order → robot starts going to kitchen → **order cancelled** → robot aborts and returns home.

### Scenario 5 — Multi-Table Sequential Delivery
Orders for tables [1, 2, 3] → kitchen (confirm) → table 1 (confirm) → table 2 (confirm) → table 3 (confirm) → kitchen → home.

### Scenario 6 — Multi-Table with Table 1 Timeout
Orders for tables [1, 2, 3] → kitchen (confirm) → table 1 → **no confirmation at table 1** → 40 s timeout → skip to table 2 (confirm) → table 3 (confirm) → kitchen → home.

### Scenario 7 — Multi-Table with Table 2 Cancellation
Orders for tables [1, 2, 3] → table 2 cancelled before kitchen departs → kitchen (confirm) → table 1 (confirm) → table 3 (confirm) → kitchen → home.

## State Machine

```
IDLE → GOING_TO_KITCHEN → WAITING_AT_KITCHEN → GOING_TO_TABLE → DELIVERING_FOOD
                                                        ↑              ↓
                                                        └── next table ┘
                                                               ↓ (no more tables)
                                              RETURNING_TO_KITCHEN_BEFORE_HOME → RETURNING_HOME → IDLE
```

## ROS2 Interface

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/butler/state` | `std_msgs/String` | JSON: state, current_table, pending/active orders |
| `/butler/status` | `std_msgs/String` | Human-readable status messages |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/butler/orders` | `std_msgs/Int32MultiArray` | Table IDs for new orders, e.g. `[1, 2, 3]` |
| `/butler/cancel_order` | `std_msgs/Int32` | Cancel by table ID (`-1` = cancel all) |

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/butler/confirm_at_kitchen` | `std_srvs/Empty` | Confirm food is ready at kitchen |
| `/butler/confirm_at_table` | `std_srvs/Empty` | Confirm customer received food at table |

## Package Structure

```
butler_delivery/
├── butler_delivery/
│   ├── delivery_manager.py      # Nav2-integrated state machine
│   ├── scenario_runner.py       # Automated scenario execution
│   ├── interactive_client.py    # CLI for manual control
│   ├── order_publisher.py       # Simple order publisher
│   └── robot_visualizer.py      # RViz marker publisher
├── launch/
│   ├── butler_delivery.launch.py    # Full system launch (Gazebo + Nav2 + manager)
│   ├── spawn_turtlebot.launch.py    # TurtleBot3 SDF spawn
│   └── visualization_only.launch.py # RViz only
├── config/
│   ├── restaurant_nav2_params.yaml  # Nav2 params (AMCL, DWB, NavFn)
│   ├── butler_rviz.rviz            # RViz config
│   └── delivery_visualization.rviz
├── maps/
│   ├── restaurant.pgm               # Occupancy grid (280×240 px, 0.05 m/px)
│   └── restaurant.yaml
├── worlds/
│   └── restaurant.world             # Gazebo Classic 12×10 m restaurant
├── models/
│   ├── kitchen.sdf
│   └── table.sdf
├── setup.py
└── package.xml
```

## Prerequisites

- ROS2 Humble
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_simulations`)
- Nav2 (`nav2_bringup`, `nav2_bt_navigator`, etc.)
- Gazebo Classic

## Installation

```bash
cd ~/bumperbot_ws/src
# Clone this package (or copy butler_delivery/ here)

cd ~/bumperbot_ws
colcon build --packages-select butler_delivery
source install/setup.bash
```

## Usage

### 1. Launch the Full System

```bash
export TURTLEBOT3_MODEL=waffle
source ~/bumperbot_ws/install/setup.bash

# Launch Gazebo + Nav2 + delivery manager
ros2 launch butler_delivery butler_delivery.launch.py
```

### 2. Run a Scenario (automated)

In a second terminal:
```bash
source ~/bumperbot_ws/install/setup.bash
ros2 run butler_delivery scenario_runner <1-7>
```

Example: `ros2 run butler_delivery scenario_runner 6`

### 3. Interactive Control (manual)

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 run butler_delivery interactive_client
```

Commands: `order 1 2 3`, `confirm_kitchen`, `confirm_table`, `cancel`, `status`, `state`

### 4. Monitor

```bash
ros2 topic echo /butler/state    # JSON state
ros2 topic echo /butler/status   # Human-readable messages
```

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kitchen_timeout` | 60 s | Max wait for kitchen confirmation |
| `table_confirmation_timeout` | 40 s | Max wait for table confirmation |
| `check_interval` | 0.2 s | State machine loop rate |

## Robot Positions

| Location | Coordinates (x, y) |
|----------|-------------------|
| Home | (0.0, 0.0) |
| Kitchen | (2.8, 0.0) |
| Table 1 | (-2.0, 2.0) |
| Table 2 | (-2.0, -1.5) |
| Table 3 | (0.0, -1.5) |
- Global map
- Grid for reference

## Implementation Details

### State Machine Logic

The delivery manager uses a hierarchical state machine where:
- Each state transition is logged
- Timeouts are checked every 100ms
- Services allow external intervention
- Orders are queued and processed sequentially

### Timeout Handling

- **Kitchen timeout**: Returns home without food
- **Table timeout**: Returns to kitchen, then home
- **Adjustable values**: Modify in `delivery_manager.py`

### Order Management

Orders are stored as `TableOrder` objects with:
- Table ID
- Order ID (unique timestamp)
- Status (pending, in_progress, delivered, timeout, canceled)
- Timestamps for timeout calculation
- Confirmation flags

## Future Enhancements

1. Integration with actual Nav2 navigation (currently simulated)
2. Order priority handling
3. Dynamic table position discovery
4. Obstacle avoidance logic
5. Real-time order status API
6. Integration with POS system
7. Energy-efficient path planning
8. Multi-robot coordination

## Testing

Run the test scenarios to verify functionality:

```bash
# Terminal 1: Launch system
ros2 launch butler_delivery butler_delivery.launch.py

# Terminal 2: Send orders
ros2 run butler_delivery order_publisher 1

# Terminal 3: Confirm actions
ros2 service call /butler/confirm_at_kitchen std_srvs/srv/Empty
ros2 service call /butler/confirm_at_table std_srvs/srv/Empty
```
