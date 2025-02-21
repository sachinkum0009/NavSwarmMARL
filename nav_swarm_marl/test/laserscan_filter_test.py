import rclpy
import unittest
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_swarm_marl.lib.utils import filter_laserscan
from time import sleep

class TestLaserScanFilter(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_laserscan_filter')
        self.publisher = self.node.create_publisher(LaserScan, '/scan_in', 10)
        self.subscription = self.node.create_subscription(
            LaserScan,
            '/scan_out',
            self.scan_callback,
            10
        )
        self.received_msg = None

    def scan_callback(self, msg):
        self.received_msg = msg

    def create_laserscan_msg(self):
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -1.57  # -90 degrees
        msg.angle_max = 1.57   # 90 degrees
        msg.angle_increment = 0.0523  # Approx. 1 degree per step
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [1.0] * int((msg.angle_max - msg.angle_min) / msg.angle_increment + 1)
        return msg

    def test_laserscan_filter(self):
        test_msg = self.create_laserscan_msg()

        print("Range of test_msg: ", len(test_msg.ranges))
        self.publisher.publish(test_msg)
        
        timeout = 5.0  # seconds
        end_time = self.node.get_clock().now().nanoseconds / 1e9 + timeout
        while self.received_msg is None and self.node.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        self.assertIsNotNone(self.received_msg, "No LaserScan message received on /scan_out")
        self.assertLess(len(self.received_msg.ranges), len(test_msg.ranges), "Filtered LaserScan should have fewer rays")
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
