import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RobotMovement:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.locations = {
            'home': (0, 0),
            'kitchen': (5, 5),
            'table_1': (10, 0),
            'table_2': (10, 5),
            'table_3': (10, 10)
        }

    def move_to(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.locations[location][0]
        goal.target_pose.pose.position.y = self.locations[location][1]
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result

    def return_home(self):
        self.move_to('home')
