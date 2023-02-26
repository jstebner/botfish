import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PlayerMoveClassifier(Node):
    def __init__(self):
        super().__init__('perception')


def main(args=None):
    rclpy.init(args=args)

    node = PlayerMoveClassifier()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()