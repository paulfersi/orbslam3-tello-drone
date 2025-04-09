import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import threading

class OccupancyGridPlotter(Node):
    def __init__(self):
        super().__init__('occupancy_grid_plotter')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/projected_map',
            self.listener_callback,
            10
        )
        self.lock = threading.Lock()
        self.map_data = None

        # Start a separate thread for plotting
        threading.Thread(target=self.plot_loop, daemon=True).start()

    def listener_callback(self, msg: OccupancyGrid):
        self.get_logger().info(f'Received map: {msg.info.width} x {msg.info.height}')
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        with self.lock:
            self.map_data = data

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        img = None

        while rclpy.ok():
            with self.lock:
                if self.map_data is not None:
                    if img is None:
                        img = ax.imshow(self.map_data, cmap='gray', origin='lower')
                    else:
                        img.set_data(self.map_data)
                    ax.set_title('Occupancy Grid Map')
                    plt.draw()
                    plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
