#!/usr/bin/env python3

"""
Dummy Task Server
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav_swarm_inter.srv import Goal

import time

class DummyTaskServer(Node):
    def __init__(self):
        super().__init__('dummy_task_server')
        reentrant_callback_group0 = ReentrantCallbackGroup()
        reentrant_callback_group1 = ReentrantCallbackGroup()
        reentrant_callback_group2 = ReentrantCallbackGroup()
        self.task_server0 = self.create_service(Goal, "/tb3_0/task_server", self.task_service_callback, callback_group=reentrant_callback_group0)
        self.task_server1 = self.create_service(Goal, "/tb3_1/task_server", self.task_service_callback, callback_group=reentrant_callback_group1)
        self.task_server2 = self.create_service(Goal, "/tb3_2/task_server", self.task_service_callback, callback_group=reentrant_callback_group2)
        self.get_logger().info("Dummy Task Server initialized.")
        
    def task_service_callback(self, req: Goal.Request, res: Goal.Response):
        print(f"robot id {req.scenario_id}")
        time.sleep(5)
        print("sending response")
        res.success = True
        res.message = str(req.scenario_id)
        return res

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = DummyTaskServer()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
