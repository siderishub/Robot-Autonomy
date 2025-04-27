import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

class CompareOdom(Node):
    def __init__(self):
        super().__init__('compare_odom')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose_subscription = self.create_subscription(Pose2D, '/estimated_pose', self.pose_callback, 10)

        self.odom_pose = None
        self.estimated_pose = None

    def odom_callback(self, msg):
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def pose_callback(self, msg):
        self.estimated_pose = (msg.x, msg.y)

        if self.odom_pose is not None:
            error = ((self.estimated_pose[0] - self.odom_pose[0]) ** 2 +
                     (self.estimated_pose[1] - self.odom_pose[1]) ** 2) ** 0.5
            self.get_logger().info(f"Error: {error:.4f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = CompareOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
