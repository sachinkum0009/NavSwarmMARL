import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_swarm_marl.lib.utils import filter_laserscan

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laserscan_filter_node')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_in',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, '/scan_out', 10)
    
    def scan_callback(self, msg: LaserScan):
        filtered_scan = filter_laserscan(msg)
        
        self.publisher.publish(filtered_scan)
        self.get_logger().info("Published filtered LaserScan with {} rays".format(len(filter_laserscan)))


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
