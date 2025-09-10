import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import pytest
import time

def publish_scan(scan_pub, ranges):
    scan_msg = LaserScan()
    scan_msg.ranges = ranges
    scan_msg.angle_min = 0.0
    scan_msg.angle_max = 0.0
    scan_msg.angle_increment = 0.0
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.0
    scan_msg.range_min = 0.0
    scan_msg.range_max = 10.0
    scan_pub.publish(scan_msg)

def publish_cmd_vel(cmd_vel_pub, linear_x, angular_z):
    twist_msg = Twist()
    twist_msg.linear.x = linear_x
    twist_msg.angular.z = angular_z
    cmd_vel_pub.publish(twist_msg)

@pytest.fixture(scope="module")
def ros2_nodes():
    rclpy.init()
    from nav_swarm_marl.emergency_cmd_node import EmergencyCmdNode
    emergency_node = EmergencyCmdNode()
    test_node = Node('test_node')
    executor = MultiThreadedExecutor()
    executor.add_node(emergency_node)
    executor.add_node(test_node)
    yield emergency_node, test_node, executor
    executor.remove_node(emergency_node)
    executor.remove_node(test_node)
    emergency_node.destroy_node()
    test_node.destroy_node()
    rclpy.shutdown()

@pytest.mark.timeout(10)
def test_emergency_cmd_forward(ros2_nodes):
    emergency_node, test_node, executor = ros2_nodes
    cmd_vel_pub = test_node.create_publisher(Twist, 'cmd_vel', 10)
    scan_pub = test_node.create_publisher(LaserScan, 'scan', 10)
    received = {}
    def emergency_callback(msg):
        received['msg'] = msg
    emergency_sub = test_node.create_subscription(Twist, 'emergency_cmd_vel', emergency_callback, 10)
    # Publish scan with no obstacle
    publish_scan(scan_pub, [1.0]*10)
    publish_cmd_vel(cmd_vel_pub, 0.5, 0.1)
    # Spin executor to process messages
    for _ in range(20):
        executor.spin_once(timeout_sec=0.1)
        if 'msg' in received:
            break
    assert 'msg' in received, "No emergency_cmd_vel received"
    assert received['msg'].linear.x == 0.5, "Command not forwarded when no obstacle"

@pytest.mark.timeout(10)
def test_emergency_cmd_stop(ros2_nodes):
    emergency_node, test_node, executor = ros2_nodes
    cmd_vel_pub = test_node.create_publisher(Twist, 'cmd_vel', 10)
    scan_pub = test_node.create_publisher(LaserScan, 'scan', 10)
    received = {}
    def emergency_callback(msg):
        received['msg'] = msg
    emergency_sub = test_node.create_subscription(Twist, 'emergency_cmd_vel', emergency_callback, 10)
    # Publish scan with obstacle
    publish_scan(scan_pub, [0.2]*10)
    # Spin to ensure scan is processed before cmd_vel
    for _ in range(10):
        executor.spin_once(timeout_sec=0.1)
    publish_cmd_vel(cmd_vel_pub, 0.5, 0.1)
    # Spin executor to process messages
    for _ in range(20):
        executor.spin_once(timeout_sec=0.1)
        if 'msg' in received:
            break
    assert 'msg' in received, "No emergency_cmd_vel received"
    assert received['msg'].linear.x == 0.0, "Stop command not sent when obstacle detected"
