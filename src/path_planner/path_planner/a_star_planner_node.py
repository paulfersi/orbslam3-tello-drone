import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import heapq

class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('a_star_path_planner')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/projected_map', self.map_callback, 10)
        self.path_pub = self.create_publisher(MarkerArray, '/full_path', 10)
        self.grid = None

    def map_callback(self, msg):
        self.get_logger().info("Received grid map")
        self.grid = self.convert_to_2d(msg)

        start = (5, 5)   # cambia
        goal = (20, 20) # cambia

        path = self.a_star(self.grid, start, goal)

        if path:
            self.publish_markers(path, msg.info)
            self.get_logger().info("Path published as markers.")
        else:
            self.get_logger().info("No path found")

    def convert_to_2d(self, grid_msg):
        width = grid_msg.info.width
        data = grid_msg.data
        height = len(data) // width
        
        if len(data) % width != 0:
            raise ValueError("The grid data length is not divisible by the width.")
        
        # grid as list of rows
        grid = []
        for row_index in range(height):
            row_start = row_index * width
            row_end = row_start + width
            grid.append(data[row_start:row_end])
        
        return grid

    def a_star(self, grid, start, goal):
        def heuristic(a, b):
            # manhattan dist
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def neighbors(node):
            directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            result = [] # every possible neighbor
            for d in directions:
                x, y = node[0] + d[0], node[1] + d[1]
                if 0 <= x < len(grid[0]) and 0 <= y < len(grid) and grid[y][x] == 0: # (x,y) is valid neighbor and null so no obstacles
                    result.append((x, y))
            return result

        open_set = [(0, start)] # min-heap prioritized by node cost (f(n)),start node and priority=0. This stores nodes to explore
        heapq.heapify(open_set)
        came_from = {} # to keep track of node parent. {node: parent} -> this way we can go back from a node by searching for his parent as a key and so on
        cost_so_far = {start: 0} # cost to reach each node from the start {node: cost}

        while open_set:
            _, current = heapq.heappop(open_set) # node with lowest cost
            if current == goal:  # goal reached
                return self.reconstruct_path(came_from, start, goal)
            for next_node in neighbors(current):
                new_cost = cost_so_far[current] + 1 # compute new cost to reach neighbor
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]: 
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)   # f(n) = g(n) + h(n)
                    heapq.heappush(open_set, (priority, next_node))
                    came_from[next_node] = current
        return [] # no solutions

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start: # we go back from current node until we reach the start
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def publish_markers(self, path, info):
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # grid indices to map coordinates
            marker.pose.position.x = info.origin.position.x + x * info.resolution
            marker.pose.position.y = info.origin.position.y + y * info.resolution
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "path_line"
        line_marker.id = len(path) + 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.1  # Line width
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0

        for (x, y) in path:
            point = Marker().pose.position
            point.x = info.origin.position.x + x * info.resolution
            point.y = info.origin.position.y + y * info.resolution
            point.z = 0.0
            line_marker.points.append(point)

        marker_array.markers.append(line_marker)

        self.path_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
