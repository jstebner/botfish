import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stockfish import Stockfish
from time import sleep

class Engine(Node):
    def __init__(self):
        super().__init__('engine')
        self.settings = {'path': None} # TODO: make this use actual engine path idk how tho
        
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
    
    
    # TODO: this ig
    def process_ui_msg(self, msg):
        data_tokens = msg.data.split()
        if data_tokens[0] == 'ping':
            pass # i think we do not care

        elif data_tokens[0] == 'stopall':
            exit() # TODO: make this good or smthn

        else: # startup
            for item in data_tokens:
                key, val = item.split('=')
                self.settings[key] = val
            self.start_engine()


    # TODO: this ig?
    def process_player_move(self, msg):
        move = msg.data
        if not self.sf.is_move_correct(move):
            return
        self.sf.make_moves_from_current_position([move])
        
        move = self.sf.get_best_move()
        self.sf.make_moves_from_current_position([move])
        return move
        

    # TODO: this
    def process_debug_cmd(self, msg):
        cmd_tokens = msg.data.split()
        
    
    # REVIEW: this
    def start_engine(self):
        # NEEDS: engine_path, level, color
        self.sf = Stockfish(
            path = self.settings['path'],
            parameters = {
                'UCI_Elo': {'easy_dff':500, 'medi_diff':1000, 'hard_diff':2500}[self.settings['difficulty']],
                'UCI_LimitStength': True,
                'Slow Mover': 0,
                'Ponder': False
            }
        )


def main(args=None):
    rclpy.init(args=args)

    node = Engine()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()