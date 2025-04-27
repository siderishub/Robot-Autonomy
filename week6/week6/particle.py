import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.particle_subscriber = self.create_subscription(ParticleCloud, '/particle_cloud', self.particle_callback, qos_profile)
        self.localization_publisher = self.create_publisher(Pose, '/custom_localization', 10)

    def particle_callback(self, msg):
        print(f"Number of particles: {len(msg.particles)}")
        x_sum, y_sum, total_weight = 0.0, 0.0, 0.0
        for particle in msg.particles:
            weight = particle.weight
            x_sum += particle.pose.position.x * weight
            y_sum += particle.pose.position.y * weight
            total_weight += weight
        
        if total_weight > 0:
            expected_x = x_sum / total_weight
            expected_y = y_sum / total_weight
            self.get_logger().info(f'Expected Position: x={expected_x:.2f}, y={expected_y:.2f}')

            pose_msg = Pose()
            pose_msg.position.x = expected_x
            pose_msg.position.y = expected_y
            self.localization_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
