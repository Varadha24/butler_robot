import rospy
from robot_movement import RobotMovement

class OrderHandler:
    def __init__(self, movement: RobotMovement):
        self.movement = movement

    def handle_order(self, order):
        table_number = order.table_number
        self.movement.move_to('kitchen')
        confirmed = self.wait_for_confirmation('kitchen')
        
        if not confirmed:
            self.movement.return_home()
            return

        self.movement.move_to(f'table_{table_number}')
        confirmed = self.wait_for_confirmation(f'table_{table_number}')
        
        if not confirmed:
            self.movement.move_to('kitchen')
        
        self.movement.return_home()

    def wait_for_confirmation(self, location, timeout=10):
        rospy.loginfo(f"Waiting for confirmation at {location}...")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if rospy.get_param(f"{location}_confirmed", False):
                rospy.loginfo(f"Confirmation received at {location}.")
                return True
        rospy.loginfo("Timeout! No confirmation received.")
        return False
