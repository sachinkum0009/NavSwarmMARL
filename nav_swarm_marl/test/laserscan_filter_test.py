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
        rclpy.shutdown()

if __name__ == '__main__':
    pytest.main()
