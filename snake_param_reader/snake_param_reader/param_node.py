from os import name
import rclpy
from rclpy.node import Node

class TestParams(Node):

    def __init__(self):
        super().__init__('reader_params_rclpy')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('double_A', 0.0),
                ('double_B', 0.0),
                ('double_C', 0.0)
            ])
        

# The following is just to start the node
def main(args=None):
    rclpy.init(args=args)
    node = TestParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()