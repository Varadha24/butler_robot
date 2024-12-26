
from src.robot import ButlerRobot
from src.order_manager import OrderManager

def main():
    robot = ButlerRobot()
    order_manager = OrderManager(robot)

    # Example usage
    orders = [
        {"table": 1},
        {"table": 2},
        {"table": 3}
    ]

    for order in orders:
        order_manager.receive_order(order)

if __name__ == "__main__":
    main()
