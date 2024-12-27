import rospy
from robot_movement import RobotMovement

class CancellationHandler:
    def __init__(self, movement: RobotMovement):
        self.movement = movement

    def handle_cancellation(self, order):
        table_number = order.table_number
        if self.movement.current_position in ['kitchen', f'table_{table_number}']:
            self.movement.return_home()
        else:
            self.movement.move_to('kitchen')
            self.movement.return_home()
