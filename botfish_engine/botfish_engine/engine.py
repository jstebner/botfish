import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stockfish import Stockfish
from time import sleep

class Engine(Node):
    def __init__(self):
        super().__init__('engine')
        self.gamestate_pub = self.create_publisher(
            String,
            'gamestate',
            10
        )
        self.bot_move_pub = self.create_publisher(
            String,
            'bot_move',
            10
        )
        self.player_move_sub = self.create_subscription(
            String,
            'player_move',
            self.process_player_move,
            10
        )
        self.ui_msg_sub = self.create_subscription(
            String,
            'ui_msg',
            self.process_ui_msg,
            10
        )
        self.debug_cmd_sub = self.create_subscription(
            String,
            'debug_cmd',
            self.process_debug_cmd,
            10
        )
    
    # TODO: this
    def process_ui_msg(self, msg):
        data_tokens = msg.data.split()
        if data_tokens[0] == 'ping':
            pass

        elif data_tokens[0] == 'stopall':
            pass

        else: # startup
            self.settings = {
                key: val 
                for item in data_tokens for key, val in item.split('=')
            }
            self.start_engine()


    # TODO: this
    def process_player_move(self, msg):
        move = msg.data

    # TODO: this
    def process_debug_cmd(self, msg):
        cmd_tokens = msg.data.split()
    
    # TODO: this
    def start_engine(self):
        # NEEDS: engine_path, level, color, side
        pass

def main(args=None):
    rclpy.init(args=args)

    node = Engine()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()