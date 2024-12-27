
# Butler Robot

## Overview
This project is designed to implement a butler robot that can handle food delivery tasks in a restaurant setting. The robot can move from its home position to the kitchen, pick up food, deliver it to the specified table, and return to its home position. Various scenarios are handled, such as waiting for confirmation, handling multiple orders, and dealing with cancellations.

## Requirements
- ROS Noetic
- Python 3

## Setup Instructions

1. **Install ROS**: Ensure you have ROS Noetic installed on your system. Follow the installation instructions for your specific version of Ubuntu [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. **Create a ROS Workspace**:
   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

3. **Copy the Files**: Extract the contents of this package into the `src` directory of your ROS workspace.
   ```sh
   cd ~/catkin_ws/src
   unzip /path/to/butler_robot.zip
   ```

4. **Build the Workspace**:
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```

5. **Source the Workspace**:
   ```sh
   source devel/setup.bash
   ```

6. **Run the Launch File**: Use the provided launch file to start the robot's operation.
   ```sh
   roslaunch butler_robot butler_robot.launch
   ```

## Technical Documentation

### ROS Nodes
The system consists of the following ROS nodes:

1. **order_listener**: Listens for incoming orders and triggers the robot's movement.
2. **movement_controller**: Controls the robot's movements based on the current task.
3. **confirmation_listener**: Listens for confirmation signals from the kitchen or tables.
4. **timeout_handler**: Manages timeouts and ensures the robot returns to home if not attended.

### PROJECT STRUCTURE:
butler_robot/
├── launch/
│   └── robot_butler.launch
├── src/
│   ├── robot_butler_node.py
│   ├── robot_movement.py
│   ├── order_handler.py
│   ├── confirmation_handler.py
│   ├── cancellation_handler.py
│   └── utils.py
├── CMakeLists.txt
└── package.xml

### Code File
The `src/robot_butler_node.py` file starts all necessary nodes and sets required parameters.
The `src/robot_movement.py` This script contains functions to control the robot's movement.
The `src/order_handler.py` Manages incoming orders, including single and multiple orders.
The `src/confirmation_handler.py` Manages confirmations from the kitchen and tables.
The `src/cancellation_handler.py` Handles order cancellations at different stages.
The `src/utils.py` Contains utility functions used across different modules.

### Launch File
The `butler_robot.launch` file starts all necessary nodes and sets required parameters.

## Scenarios Handled

1. **Simple Delivery**: Move from home to kitchen, then to table, and back to home.
2. **Wait for Confirmation**: Waits for confirmation at the kitchen or table before proceeding.
3. **Timeout Handling**: Returns to home if no confirmation is received within a timeout period.
4. **Cancellation Handling**: Returns to kitchen and then home if the task is canceled.
5. **Multiple Orders**: Handles delivery to multiple tables in a single trip.
6. **Conditional Delivery**: Skips tables or returns to the kitchen based on confirmation or cancellation.

## Notes
- Ensure all dependencies are installed and configured correctly.
- Modify the source code to adapt to specific hardware or requirements.

## Contact
For further assistance, contact Varadha24.
