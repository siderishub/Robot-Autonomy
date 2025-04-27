#from geometry_msgs.msg import Pose

        # self.subscription = self.create_subscription(
        #     Pose,
        #     '/custom_localization',  # Subscribe to custom localization topic
        #     self.localization_callback,
        #     10)

import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        # Map parameters
        self.map_width = 500  # Grid cells
        self.map_height = 500  # Grid cells
        self.resolution = 0.05 # Meters per cell
        self.origin_x = -self.map_width * self.resolution / 2  # Center X
        self.origin_y = -self.map_height * self.resolution / 2  # Center Y

        self.tf_broadcaster = TransformBroadcaster(self)

        # Occupancy grid (-1 = unknown, 0 = free, 100 = occupied)
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        # Robot pose (updated from odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Orientation (radians)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/show_map', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Timer to publish map periodically
        self.create_timer(1.0, self.publish_map)  # 1 Hz

    def odom_callback(self, msg):
        """Update robot position and orientation from odometry data."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation  # Keep original orientation

        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(f"Updated Pose: x={self.robot_x:.2f}, y={self.robot_y:.2f}, theta={self.robot_theta:.2f}")

    def quaternion_to_yaw(self, q: Quaternion):
        """Convert quaternion to yaw (rotation around z-axis)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg: LaserScan):
        """Transform LIDAR scan data into map coordinates and update the occupancy grid."""
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Compute hit point in global coordinates
                x_rel = r * math.cos(angle)
                y_rel = r * math.sin(angle)
                x_hit = self.robot_x + x_rel * math.cos(self.robot_theta) - y_rel * math.sin(self.robot_theta)
                y_hit = self.robot_y + x_rel * math.sin(self.robot_theta) + y_rel * math.cos(self.robot_theta)

                # Convert world coordinates to map indices
                x0 = int((self.robot_x - self.origin_x) / self.resolution)
                y0 = int((self.robot_y - self.origin_y) / self.resolution)
                x1 = int((x_hit - self.origin_x) / self.resolution)
                y1 = int((y_hit - self.origin_y) / self.resolution)

                # Mark free cells along the ray
                for x_cell, y_cell in self.bresenham(x0, y0, x1, y1)[:-1]:  # Exclude the final cell (hit)
                    if 0 <= x_cell < self.map_width and 0 <= y_cell < self.map_height:
                        if self.map_data[y_cell, x_cell] != 100:  # Don't overwrite occupied cells
                            self.map_data[y_cell, x_cell] = 0

                # Mark the hit cell as occupied
                if 0 <= x1 < self.map_width and 0 <= y1 < self.map_height:
                    self.map_data[y1, x1] = min(self.map_data[y1, x1] + 10, 100)
            
            angle += msg.angle_increment

        self.get_logger().info("Updated Map with LIDAR Data")
    
    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to get cells between two points."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x1, y1))
        return cells


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
        self.get_logger().info("Published Map")

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



