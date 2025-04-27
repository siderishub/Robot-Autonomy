import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
import random
import math

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/prm_markers', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.occupancy_grid = None
        self.prm_nodes = []
        self.resolution = None
        self.map_origin = None

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.occupancy_grid = OccupancyGridWrapper(map_data, self.resolution, self.map_origin)
        
        self.generate_prm_nodes()
        self.evaluate_and_navigate()
    
    def generate_prm_nodes(self, num_nodes=10):
        self.prm_nodes = []
        free_space = np.argwhere(self.occupancy_grid.map_data == 0)
        selected_indices = np.random.choice(len(free_space), num_nodes, replace=False)
        for idx in selected_indices:
            row, col = free_space[idx]
            x, y = self.occupancy_grid.grid_to_world(row, col)
            self.prm_nodes.append((x, y))
        self.publish_prm_markers()
    
    def publish_prm_markers(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.prm_nodes):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
    
    def evaluate_and_navigate(self):
        best_node = None
        best_info_gain = -1
        
        for x, y in self.prm_nodes:
            node = NodeObject(x, y)
            info_gain = self.occupancy_grid.compute_visible_unknown(node)
            if info_gain > best_info_gain:
                best_info_gain = info_gain
                best_node = (x, y)
        
        if best_node:
            self.navigate_to_pose(best_node)
    
    def navigate_to_pose(self, target):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.nav_to_pose_client.wait_for_server()
        self.nav_to_pose_client.send_goal_async(goal_msg)

class OccupancyGridWrapper:
    def __init__(self, map_data, resolution, origin):
        self.map_data = map_data
        self.resolution = resolution
        self.origin = origin
        self.height, self.width = map_data.shape
    
    def compute_visible_unknown(self, node, max_distance=3):
        directions = 36
        max_cells = int(max_distance / self.resolution)
        total_unknown = 0
        for d in range(directions):
            angle = 2 * math.pi * d / directions
            total_unknown += self.ray_cast_unknown(node, angle, max_cells)
        return total_unknown
    
    def ray_cast_unknown(self, node, angle, max_cells):
        count = 0
        for step in range(1, max_cells + 1):
            rx = node.x + step * self.resolution * math.cos(angle)
            ry = node.y + step * self.resolution * math.sin(angle)
            col, row = self.world_to_grid(rx, ry)
            if not (0 <= row < self.height and 0 <= col < self.width):
                break
            val = self.map_data[row, col]
            if val == -1:
                count += 1
            elif val == 100:
                break
        return count
    
    def world_to_grid(self, x, y):
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        return col, row
    
    def grid_to_world(self, row, col):
        x = col * self.resolution + self.origin[0]
        y = row * self.resolution + self.origin[1]
        return x, y

class NodeObject:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
