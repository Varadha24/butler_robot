
from src.robot import ButlerRobot

class OrderManager:
    def __init__(self, robot: ButlerRobot):
        self.robot = robot

    def receive_order(self, order):
        table_number = order["table"]
        self.handle_order(table_number)

    def handle_order(self, table_number):
        self.robot.move_to("kitchen")

        if not self.robot.wait_for_confirmation():
            self.robot.return_home()
            return

        self.robot.move_to(f"table {table_number}")

        if not self.robot.wait_for_confirmation():
            self.robot.move_to("kitchen")
            self.robot.return_home()
            return

        self.robot.return_home()
