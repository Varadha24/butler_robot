import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from utils import log_info

class RobotMovement:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.locations = {
            'home': (0.0, 0.0),
            'kitchen': (5.0, 5.0),
            'table_1': (10.0, 0.0),
            'table_2': (10.0, 5.0),
            'table_3': (10.0, 10.0)
        }

    def move_to(self, location):
        if location not in self.locations:
            log_info(f"Location {location} not found.")
            return False
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.locations[location][0]
        goal.target_pose.pose.position.y = self.locations[location][1]
        goal.target_pose.pose.orientation.w = 1.0
        
        log_info(f"Moving to {location}: {self.locations[location]}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        
        if result:
            log_info(f"Reached {location}.")
            return True
        else:
            log_info(f"Failed to reach {location}.")
            return False

    def return_home(self):
        self.move_to('home')

