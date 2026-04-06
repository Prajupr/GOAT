# Butler Robot Delivery System

A ROS2-based delivery management system for a TurtleBot3 acting as a butler robot in a restaurant setting.

## Overview

This system implements a state machine for managing food deliveries from a kitchen to customer tables, handling complex scenarios including timeouts, cancellations, and multiple concurrent orders.

## Delivery Scenarios

The system handles the following delivery scenarios as per the assessment:

### Scenario 1: Basic Single Delivery
- Robot receives order for a table
- Goes to kitchen to collect food (confirmation required)
- Goes to table to deliver food (confirmation required)
- Returns to home position
- **No confirmation needed either from kitchen or customer table**

### Scenario 2: Delivery with Kitchen Timeout
- Robot goes to kitchen and waits for food
- **If no confirmation within 30 seconds, robot returns home without food**
- Suitable for busy kitchen situations

### Scenario 3: Delivery with Table Confirmation Timeout
- Robot reaches the table with food
- **If customer doesn't confirm receipt within 20 seconds, robot goes back to kitchen, then home**
- Useful when customer is not at table

### Scenario 4: Task Cancellation
- Order is cancelled while robot is processing it
- **If cancelled at kitchen:** Robot returns directly to home
- **If cancelled on way to table:** Robot returns to home
- **If cancelled during delivery:** Robot returns to kitchen, then home

### Scenario 5: Multiple Orders Processing
- Robot receives multiple orders for different tables (1, 2, 3)
- Collects all orders from kitchen
- Delivers to each table sequentially
- Returns home after all deliveries

### Scenario 6: Multiple Orders with Selective Skipping
- Robot receives orders for tables 1, 2, 3
- Handle normal confirmations
- **If one order (e.g., table 2) is cancelled:** Skip that table and deliver to others
- Return to kitchen before going home

### Scenario 7: Multiple Orders with Cancellation
- Similar to scenario 5/6 but focused on handling cancellation logic
- **Cancel one order mid-delivery**: Robot skips that table and delivers to remaining tables

## States

The delivery manager operates with the following states:

```
IDLE -> GOING_TO_KITCHEN -> WAITING_AT_KITCHEN -> COLLECTING_FOOD 
  -> GOING_TO_TABLE -> DELIVERING_FOOD -> RETURNING_HOME -> IDLE
```

## Topics

### Published Topics

- `/butler/state` (String) - Current state in JSON format
- `/butler/status` (String) - Human-readable status messages

### Subscribed Topics

- `/butler/orders` (Int32MultiArray) - Table IDs for new orders (e.g., [1, 2, 3])

## Services

- `/butler/confirm_at_kitchen` (Empty) - Confirm food is ready
- `/butler/confirm_at_table` (Empty) - Confirm customer received food
- `/butler/cancel_order` (Empty) - Cancel current order

## Installation

1. Clone or create the package:
```bash
cd ~/bumperbot_ws/src
ros2 pkg create butler_delivery --build-type ament_python
```

2. Copy the delivery manager files:
```bash
cp delivery_manager.py ~/bumperbot_ws/src/butler_delivery/butler_delivery/
cp order_publisher.py ~/bumperbot_ws/src/butler_delivery/butler_delivery/
```

3. Build the package:
```bash
cd ~/bumperbot_ws
colcon build --packages-select butler_delivery
```

## Usage

### Launch the System

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Source the workspace
source ~/bumperbot_ws/install/setup.bash

# Launch butler delivery with TurtleBot3
ros2 launch butler_delivery butler_delivery.launch.py use_nav2:=false use_rviz:=false
```

### Interactive Client

Open another terminal and run:

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 run butler_delivery interactive_client
```

Then use commands like:
- `order 1 2 3`
- `confirm_kitchen`
- `confirm_table`
- `cancel`
- `status`
- `state`

### Send Orders

In a new terminal:

```bash
source ~/bumperbot_ws/install/setup.bash

# Scenario 1: Single order to table 1
ros2 run butler_delivery order_publisher 1

# Scenario 5: Multiple orders
ros2 run butler_delivery order_publisher 5

# Scenario 6: Multiple with handling
ros2 run butler_delivery order_publisher 6
```

### Interact with the Delivery

In a new terminal, confirm food at kitchen:

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 service call /butler/confirm_at_kitchen std_srvs/srv/Empty
```

Confirm delivery at table:

```bash
ros2 service call /butler/confirm_at_table std_srvs/srv/Empty
```

Cancel current order:

```bash
ros2 service call /butler/cancel_order std_srvs/srv/Empty
```

## Configuration

Key parameters (configurable in delivery_manager.py):

- `kitchen_timeout`: 30 seconds - Time to wait for kitchen confirmation
- `delivery_timeout`: 60 seconds - Time for delivery operation
- `table_confirmation_timeout`: 20 seconds - Time to wait for table confirmation

## Robot Positions

**Home Position:** (0.0, 0.0, 0.0)
**Kitchen Position:** (1.0, 0.0, 0.0)
**Table 1 Position:** (-0.5, 0.5, 0.0)
**Table 2 Position:** (-0.5, -0.5, 0.0)
**Table 3 Position:** (1.5, -0.5, 0.0)

## Monitoring

Watch the robot state and status:

```bash
# Monitor state updates (JSON)
ros2 topic echo /butler/state

# Monitor status messages
ros2 topic echo /butler/status
```

## RViz Visualization

The system automatically launches RViz with:
- Robot model (TurtleBot3 WAFFLE)
- LaserScan visualization
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
