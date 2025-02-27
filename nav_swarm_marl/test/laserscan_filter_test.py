#!/usr/bin/env python3

import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from nav_swarm_marl.laserscan_filter_node import LaserScanFilter

def test_publisher_creation():
    rclpy.init()
    try:
        node = LaserScanFilter()
        assert node.get_name() == 'laserscan_filter_node'
        assert hasattr(node, 'publisher')
        assert node.publisher.topic_name == '/scan_out'
    finally:
        node.destroy_node()
        rclpy.shutdown()

@pytest.fixture(scope="module")
def rclpy_node():
    rclpy.init()
    node = LaserScanFilter()
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def publisher(rclpy_node):
    return rclpy_node.create_publisher(LaserScan, '/scan_in', 10)

@pytest.fixture
def subscription(rclpy_node):
    received_msg = []
    
    def scan_callback(msg):
        received_msg.append(msg)
    
    rclpy_node.create_subscription(LaserScan, '/scan_out', scan_callback, 10)
    return received_msg

@pytest.fixture
def create_laserscan_msg():
    def _create_msg():
        msg = LaserScan()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
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
    return _create_msg

def test_laserscan_filter(rclpy_node, publisher, subscription, create_laserscan_msg):
    test_msg = create_laserscan_msg()
    print("data published with ranges: ", len(test_msg.ranges))
    publisher.publish(test_msg)
    
    timeout = 5.0  # seconds
    end_time = (rclpy_node.get_clock().now().nanoseconds / 1e9) + timeout


    while (rclpy_node.get_clock().now().nanoseconds / 1e9) < end_time:
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

#     assert subscription, "No LaserScan message received on /scan_out"
    
#     received_scan = subscription[0]  # Ensure we have a valid received message
#     assert isinstance(received_scan, LaserScan), "Received message is not a LaserScan"

#     assert hasattr(received_scan, 'ranges'), "Filtered LaserScan message has no 'ranges' attribute"
#     assert isinstance(received_scan.ranges, list), "'ranges' should be a list"
#     assert len(received_scan.ranges) < len(test_msg.ranges), "Filtered LaserScan should have fewer rays"

if __name__ == '__main__':
    pytest.main()
