import rclpy
from rclpy.node import Node
from tello_msgs.msg import FlightData
import tkinter as tk
from tkinter import ttk

class DroneInfoGui(Node):
    def __init__(self):
        super().__init__('drone_info_gui_node')

        self.window = tk.Tk()
        self.window.title("Drone Flight Info")

        self.pitch_label = ttk.Label(self.window, text="Pitch: 0°")
        self.pitch_label.pack()

        self.roll_label = ttk.Label(self.window, text="Roll: 0°")
        self.roll_label.pack()

        self.yaw_label = ttk.Label(self.window, text="Yaw: 0°")
        self.yaw_label.pack()

        self.height_label = ttk.Label(self.window, text="Height: 0 cm")
        self.height_label.pack()

        self.battery_label = ttk.Label(self.window, text="Battery: 0%")
        self.battery_label.pack()

        self.subscription = self.create_subscription(
            FlightData,
            '/flight_data',
            self.flight_data_callback,
            10
        )

    def flight_data_callback(self, msg):
        self.pitch_label.config(text=f"Pitch: {msg.pitch}°")
        self.roll_label.config(text=f"Roll: {msg.roll}°")
        self.yaw_label.config(text=f"Yaw: {msg.yaw}°")
        self.height_label.config(text=f"Height: {msg.h} cm")
        self.battery_label.config(text=f"Battery: {msg.bat}%")

    def run(self):
        def update_ros():
            rclpy.spin_once(self)
            self.window.after(100, update_ros)  # Call update_ros again after 100 ms

        update_ros()
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    drone_info_gui = DroneInfoGui()
    drone_info_gui.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
