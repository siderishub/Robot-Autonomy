import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose2D
import math

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Pose2D, '/estimated_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'my_odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.x
        transform.transform.translation.y = msg.y
        transform.transform.rotation.z = math.sin(msg.theta / 2.0)
        transform.transform.rotation.w = math.cos(msg.theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
