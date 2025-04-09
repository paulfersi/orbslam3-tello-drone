import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import matplotlib.pyplot as plt
import threading

class OccupancyGridPlotter(Node):
    def __init__(self):
        super().__init__('occupancy_grid_plotter')

        self.map_data = None
        self.map_info = None
        self.lock = threading.Lock()

        self.camera_pose = None

        self.create_subscription(OccupancyGrid, '/projected_map', self.map_callback, 10)
        self.create_subscription(Odometry, '/Odometry/OrbSlam/Odom', self.odom_callback, 10)

        threading.Thread(target=self.plot_loop, daemon=True).start()

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info_once(f"Received map: {msg.info.width} x {msg.info.height}")
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        with self.lock:
            self.map_data = data
            self.map_info = msg.info

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.camera_pose = msg.pose.pose.position

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        img = None
        camera_dot = None

        while rclpy.ok():
            with self.lock:
                if self.map_data is not None:
                    if img is None:
                        img = ax.imshow(self.map_data, cmap='gray', origin='lower')
                    else:
                        img.set_data(self.map_data)

                    if self.camera_pose and self.map_info:
                        # Convert world -> map pixel coordinates
                        x_map = (self.camera_pose.x - self.map_info.origin.position.x) / self.map_info.resolution
                        y_map = (self.camera_pose.y - self.map_info.origin.position.y) / self.map_info.resolution

                        if 0 <= int(x_map) < self.map_info.width and 0 <= int(y_map) < self.map_info.height:
                            if camera_dot:
                                camera_dot.set_data(x_map, y_map)
                            else:
                                camera_dot, = ax.plot(x_map, y_map, 'ro')  # red dot

                    ax.set_title('Occupancy Grid Map with Camera Position')
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
