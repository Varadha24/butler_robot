import rospy
from robot_movement import RobotMovement

class ConfirmationHandler:
    def __init__(self, movement: RobotMovement):
        self.movement = movement

    def handle_confirmation(self, location):
        rospy.set_param(f"{location}_confirmed", True)
