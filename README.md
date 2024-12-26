
# Butler Robot Project

## Overview
This project simulates a butler robot that delivers food to tables in a restaurant. The robot follows specific tasks based on the orders received.

## Directory Structure
```
butler_robot/
├── launch/
│   └── robot_launch.py
├── src/
│   ├── main.py
│   ├── robot.py
│   ├── order_manager.py
│   └── utils.py
└── README.md
```

## How to Run
1. Navigate to the `butler_robot` directory.
2. Run the `robot_launch.py` script to start the application:
   ```sh
   python launch/robot_launch.py
   ```

## Functionality
- When an order is received, the robot moves from its home position to the kitchen and then to the specified table.
- If no one attends to the robot within a certain timeout period, it returns home.
- The robot handles multiple orders, cancellations, and confirmations as described in the main code.

## Example Usage
Modify the `orders` list in `src/main.py` to simulate different order scenarios and test the robot's behavior.
