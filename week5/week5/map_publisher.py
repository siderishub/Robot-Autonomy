import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan
import math

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        
        # Map parameters
        self.map_width = 20  # cells
        self.map_height = 20  # cells
        self.resolution = 0.5  # meters per cell
        self.origin_x = -5.0  # meters
        self.origin_y = -5.0  # meters
        
        # Create 2D map matrix
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.add_random_obstacles()
        
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/show_map', 10)
        self.timer = self.create_timer(2.0, self.publish_map)  # 0.5 Hz
        
        # Static Transform Publisher
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()
        
        # LIDAR Subscription
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

    def add_random_obstacles(self):
        """Populate random cells with obstacles."""
        num_obstacles = int(0.1 * self.map_width * self.map_height)
        obstacle_indices = np.random.choice(self.map_width * self.map_height, num_obstacles, replace=False)
        for idx in obstacle_indices:
            x, y = divmod(idx, self.map_width)
            self.map_data[y, x] = 100  # Mark as occupied

    def publish_map(self):
        """Publish the occupancy grid map."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(msg)
        self.get_logger().info("Published map")
    
    def publish_static_transform(self):
        """Publish a static transform from map to base_link."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Published static transform")
    
    def lidar_callback(self, msg: LaserScan):
        """Convert LIDAR scan to map and update the occupancy grid."""
        self.map_data.fill(0)  # Reset to empty
        #self.add_random_obstacles()  # Keep initial obstacles
        
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = int((r * math.cos(angle) - self.origin_x) / self.resolution)
                y = int((r * math.sin(angle) - self.origin_y) / self.resolution)
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    self.map_data[y, x] = 100
            angle += msg.angle_increment
        
        self.publish_map()
        self.get_logger().info("Updated map with LIDAR data")


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
