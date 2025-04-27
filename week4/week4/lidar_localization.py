import rclpy
import numpy as np
import open3d as o3d  # Open3D for ICP
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from scipy.spatial.transform import Rotation as R

class LidarLocalization(Node):
    def __init__(self):
        super().__init__('lidar_localization')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pose_publisher = self.create_publisher(Pose2D, '/estimated_pose', 10)
        
        self.prev_scan = None
        self.pose = Pose2D()

    def scan_callback(self, msg):
        # Convert scan data from polar to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Remove invalid LIDAR readings
        valid_mask = np.isfinite(ranges) & (ranges > 0)  # Remove NaN, inf, and negatives
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        # Convert to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        current_scan = np.vstack((x, y)).T  # Shape (N,2)

        if self.prev_scan is not None:
            transformation = self.estimate_transform(self.prev_scan, current_scan)
            if transformation is not None:
                dx, dy, dyaw = transformation
                self.pose.x += dx
                self.pose.y += dy
                self.pose.theta += dyaw
                self.pose_publisher.publish(self.pose)

        self.prev_scan = current_scan

    def estimate_transform(self, prev_scan, current_scan):
        """
        Uses ICP to estimate the transformation between two LIDAR scans.
        """

        # Convert (Nx2) numpy arrays to (Nx3) by adding a zero z-axis
        prev_scan_3d = np.hstack((prev_scan, np.zeros((prev_scan.shape[0], 1))))
        current_scan_3d = np.hstack((current_scan, np.zeros((current_scan.shape[0], 1))))

        # Convert numpy arrays to Open3D point clouds
        source = o3d.geometry.PointCloud()
        target = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(prev_scan_3d)
        target.points = o3d.utility.Vector3dVector(current_scan_3d)

        # Perform ICP scan matching
        threshold = 0.1  # Maximum correspondence distance
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())

        # Extract transformation matrix
        transformation = reg_p2p.transformation
        dx = transformation[0, 3]
        dy = transformation[1, 3]
        dyaw = np.arctan2(transformation[1, 0], transformation[0, 0])  # Extract yaw angle

        return dx, dy, dyaw

def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
