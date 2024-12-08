from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node 
import numpy as np
import matplotlib.pyplot as plt

class OccupancyGridPlotter(Node):
    def __init__(self):
        super().__init__('occupancy_grid_plotter')
        
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid,'/projected_map',self.callback,10)

    def callback(self,msg):
        self.plot_occupancy_grid(msg)

    def plot_occupancy_grid(self,occupancy_grid):
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data,dtype=np.int8)

        grid = data.reshape((height,width))

        grid = np.where(grid==.1,50,grid)

        plt.figure(figsize=(8,8))
        plt.imshow(grid,cmap='gray',origin='lower')
        plt.title("Occupancy Grid")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()

        


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPlotter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()