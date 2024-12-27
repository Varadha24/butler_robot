import rospy
from robot_movement import RobotMovement
from order_handler import OrderHandler
from confirmation_handler import ConfirmationHandler
from cancellation_handler import CancellationHandler

class ButlerRobotNode:
    def __init__(self):
        rospy.init_node('butler_robot_node')
        self.movement = RobotMovement()
        self.order_handler = OrderHandler(self.movement)
        self.confirmation_handler = ConfirmationHandler(self.movement)
        self.cancellation_handler = CancellationHandler(self.movement)
        rospy.Subscriber('orders', Order, self.order_callback)
        rospy.Subscriber('cancellations', Order, self.cancellation_callback)
        rospy.spin()

    def order_callback(self, order):
        self.order_handler.handle_order(order)

    def cancellation_callback(self, order):
        self.cancellation_handler.handle_cancellation(order)

if __name__ == '__main__':
    try:
        ButlerRobotNode()
    except rospy.ROSInterruptException:
        pass
