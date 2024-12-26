
import time

class ButlerRobot:
    def __init__(self):
        self.position = "home"

    def move_to(self, position):
        print(f"Moving to {position}...")
        time.sleep(2)  # Simulate time taken to move
        self.position = position
        print(f"Arrived at {position}.")

    def wait_for_confirmation(self, timeout=10):
        print(f"Waiting for confirmation at {self.position}...")
        time.sleep(timeout)  # Simulate waiting time
        print("Timeout! No confirmation received.")
        return False  # Simulate no confirmation received

    def return_home(self):
        print("Returning to home position...")
        self.move_to("home")
