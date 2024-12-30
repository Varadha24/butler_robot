
# Butler Robot

## Overview

This project is designed to implement a butler robot that can handle food delivery tasks in a restaurant setting. The robot can move from its home position to the kitchen, pick up food, deliver it to the specified table, and return to its home position. Various scenarios are handled, such as waiting for confirmation, handling multiple orders, and dealing with cancellations.

## Features

Autonomous navigation to the kitchen and tables.

Order confirmation handling with timeout mechanisms.

Cancellation handling at different stages of the task.

Efficient processing of multiple orders.

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
   
5. **Run the Project**: Start the ROS Master.
   ```sh
   roscore
   ```

6. **Source the Workspace**:
   ```sh
   source devel/setup.bash
   ```
   
7. **Launch the Butler Robot node**: Use the provided launch file to start the robot's operation.
   ```sh
   roslaunch butler_robot butler_robot.launch
   ```
   
## Configuration

1. **Locations Configuration**: Define the coordinates for home, kitchen, and tables in a configuration file (e.g., `locations.yaml`).
   ```sh
   home: [0.0, 0.00]
   kitchen: [5.0, 5.0]
   table_1: [10.0, 0.0]
   table_2: [10.0, 5.0]
   table_3: [10.0, 10.0]
   ```

## Technical Documentation

### ROS Nodes

The system consists of the following ROS nodes:

1. **robot_butler_node**: Handles the main control flow of the robot, managing tasks such as receiving orders, delegating movements, and coordinating confirmations and cancellations.
2. **robot_movement**: Controls the robot's navigation to predefined locations including the kitchen, tables, and home position using ROS navigation stack.
3. **order_handler**: Manages the queue and execution of incoming orders, ensuring efficient processing of single and multiple orders.
4. **confirmation_handler**: Handles the confirmation process from the kitchen and tables, implementing timeout mechanisms to manage unattended tasks.
5. **cancellation_handler**: Manages the logic for order cancellations, ensuring the robot returns to the appropriate positions based on the stage of the task.
6. **utils**: Provides utility functions for logging, configuration loading, and other common tasks used across the project's modules.

### PROJECT STRUCTURE:

  ```sh
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
├── LICENSE
└── package.xml
```

### Code File
`src/robot_butler_node.py` The main script that controls the robot's behavior based on incoming orders.

`src/robot_movement.py` This script contains functions to control the robot's movement.

`src/order_handler.py` Manages incoming orders, including single and multiple orders.

`src/confirmation_handler.py` Manages confirmations from the kitchen and tables.

`src/cancellation_handler.py` Handles order cancellations at different stages.

`src/utils.py` Contains utility functions used across different modules.

### Build Files
`CMakeLists.txt`: Defines the build configuration for the project.

`package.xml`: Specifies the package dependencies and metadata.

### Launch File
`butler_robot.launch` file starts all necessary nodes and sets required parameters.

## Scenarios Handled

1. **Simple Delivery**: Move from home to kitchen, then to table, and back to home.
2. **Wait for Confirmation**: Waits for confirmation at the kitchen or table before proceeding.
3. **Timeout Handling**: Returns to home if no confirmation is received within a timeout period.
4. **Cancellation Handling**: Returns to kitchen and then home if the task is canceled.
5. **Multiple Orders**: Handles delivery to multiple tables in a single trip.
6. **Conditional Delivery**: Skips tables or returns to the kitchen based on confirmation or cancellation.

## Functionality

**Move to Kitchen**
The robot moves from the home position to the kitchen to pick up food.

**Move to Table**
The robot moves from the kitchen to the specified table to deliver food.

**Return to Home**
After completing its delivery task, the robot returns to its home position.

**Handling Scenarios**
The robot handles various scenarios such as waiting for confirmation, handling timeouts, and managing order cancellations efficiently.
**Testing**

**Unit Testing**
Test individual functions for correctness using unit tests.

**Integration Testing**
Test the entire workflow to ensure smooth operation of the robot in a simulated environment.

## Notes
- Ensure all dependencies are installed and configured correctly.
- Modify the source code to adapt to specific hardware or requirements.

