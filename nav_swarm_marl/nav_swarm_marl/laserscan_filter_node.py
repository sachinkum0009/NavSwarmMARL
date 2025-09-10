import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_swarm_marl.lib.utils import filter_laserscan

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laserscan_filter_node')

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan_in',
            self.scan_callback,
            qos_profile
        )
        self.publisher = self.create_publisher(LaserScan, 'scan_out', qos_profile)
    
    def scan_callback(self, msg: LaserScan):
        filtered_scan = filter_laserscan(msg)

        if filtered_scan is not None:
            self.publisher.publish(filtered_scan)
            self.get_logger().info("Published filtered LaserScan with {} rays".format(len(filtered_scan.ranges)))
        else:
            self.get_logger().error("Failed to publish filtered LaserScan")


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
