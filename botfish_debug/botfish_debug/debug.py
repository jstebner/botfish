import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DebugController(Node):
    def __init__(self):
        super().__init__('debug_controller')
        self.q = list()

        self.pub = self.create_publisher(
            String,
            'debug_cmd',
            10
        )
        self.sub = self.create_subscription(
            String,
            'ui_ping',
            self.display_ping,
            10
        )

        self.main()

    def display_ping(self, msg):
        self.q.append(msg.data)

    def main(self):
        stopwords = ('exit', 'quit', 'stop', 'end', 'LET ME OUT OF THIS CHAIR YOU IDIOT')
        while True:
            while self.q:
                print(self.q.pop(0))

            cmd = input('>')
            if cmd in stopwords:
                break
            if cmd == '':
                continue

            msg = String()
            msg.data = cmd
            self.pub.publish(msg)
        
        self

def main(args=None):
    rclpy.init(args=args)

    debug = DebugController()
    rclpy.spin(debug)

    debug.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
