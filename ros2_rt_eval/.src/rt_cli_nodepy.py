import rclpy
from rclpy.node import Node
from ros2_rt_eval_dep.srv import Vector
import time

class VectorClient(Node):

    def __init__(self):
        super().__init__('rt_cli_nodepy')
        self.client = self.create_client(Vector, 'vector_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = Vector.Request()

    def send_request(self, input_vector):
        self.request.input = input_vector
        start_time = time.time()
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        end_time = time.time()
        if future.result() is not None:
            self.get_logger().info('Received response')
            elapsed_time = (end_time - start_time) * 1000  # Convert to milliseconds
            return elapsed_time
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = VectorClient()
    input_vector = [1, 2, 3, 4, 5]
    times = []

    for _ in range(100):  # Adjust the number of calls as needed
        elapsed_time = client.send_request(input_vector)
        if elapsed_time is not None:
            times.append(elapsed_time)
            print(f"Time taken: {elapsed_time:.2f} ms")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
