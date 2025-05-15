import rclpy
from rclpy.node import Node

from nav_swarm_inter.srv import Goal  # Make sure this matches your actual service definition

class DummyServiceClient(Node):
    def __init__(self):
        super().__init__('dummy_service_client')
        self.scenario_client = self.create_client(Goal, "/scenario_goal")
        while not self.scenario_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /scenario_goal service...')
        self.send_goal_request()

    def send_goal_request(self):
        request = Goal.Request()
        # Fill in request fields as needed, e.g.:
        # request.target = ...
        request.x = 1.0
        request.scenario_id = 3
        future = self.scenario_client.call_async(request)
        # rclpy.spin_until_future_complete(
        #     self, future, timeout_sec=2.0
        # )
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        self.get_logger().info(f'Response: {response}')
        # future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyServiceClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()