#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_swarm_inter.srv import Goal
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class NavigationServer(Node):
    def __init__(self):
        super().__init__(
            "navigation_server",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        reentrant_callback_group = ReentrantCallbackGroup()
        self.task_server = self.create_service(
            Goal,
            "task_server",
            self.task_server_callback,
            callback_group=reentrant_callback_group,
        )
        self.navigator = BasicNavigator()
        self.get_logger().info("waiting for nav2")
        self.navigator.waitUntilNav2Active()

    def task_server_callback(self, req: Goal.Request, res: Goal.Response):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = req.x
        goal_pose.pose.position.y = req.y
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        go_to_task = self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback.navigation_duration > 600:
                self.navigator.cancelTask()

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("goal succeeded")
            res.success = True
            res.message = "Goal succeeded"
        elif result == TaskResult.CANCELED:
            self.get_logger().info("goal canceled")
            res.success = False
            res.message = "Goal canceled"
        elif result == TaskResult.FAILED:
            self.get_logger().info("goal failed")
            res.success = False
            res.message = "Goal failed"
        else:
            res.success = False
            res.message = "Unknown result"
        return res


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = NavigationServer()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Navigation Server node stopped by user")
    finally:
        executor.remove_node(node)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
