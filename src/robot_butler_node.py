import rospy
from robot_movement import RobotMovement
from order_handler import OrderHandler
from confirmation_handler import ConfirmationHandler
from cancellation_handler import CancellationHandler
from utils import log_info
from butler_robot.msg import Order  # Assuming you have a custom Order message defined

class ButlerRobotNode:
    def __init__(self):
        rospy.init_node('robot_butler_node')
        self.movement = RobotMovement()
        self.order_handler = OrderHandler(self.movement)
        self.confirmation_handler = ConfirmationHandler()
        self.cancellation_handler = CancellationHandler(self.movement)
        
        rospy.Subscriber('orders', Order, self.order_callback)
        rospy.Subscriber('cancellations', Order, self.cancellation_callback)
        
        log_info("Butler Robot Node started.")
        rospy.spin()

    def order_callback(self, order):
        log_info(f"Received order: {order}")
        self.order_handler.handle_order(order)

    def cancellation_callback(self, order):
        log_info(f"Received cancellation for order: {order}")
        self.cancellation_handler.handle_cancellation(order)

if __name__ == '__main__':
    try:
        ButlerRobotNode()
    except rospy.ROSInterruptException:
        pass
