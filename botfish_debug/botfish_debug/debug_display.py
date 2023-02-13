import sys
import os
import io

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import chess
import chess.svg
import pygame
import cairosvg
from pygame.locals import *
from multiprocessing import Queue

SIZE = (512 + 256,1024)
WIDGET_SIZE = 512

class DebugDisplay(Node):
    PERIOD_s = 1/10 # 10 UpS
    DIRPATH = os.path.dirname(__file__)

    def __init__(self):
        pygame.init()
        super().__init__('debug_display')
        
        pygame.display.set_caption('Debug')
        self.q = Queue()
        self.screen = pygame.display.set_mode(SIZE) # testing
        self.FONT = pygame.font.SysFont('Calibri', 15)
        self.board = chess.Board()
        self.board_rendered = None
        self._render_board()
        
        self.sub = self.create_subscription(
            String,
            'update_debug_board',
            self.q.put,
            10
        )
        self.timer = self.create_timer(
            self.PERIOD_s,
            self.update
        )
    
    def _render_board(self):
        render = io.BytesIO()
        cairosvg.svg2png(
            chess.svg.board(self.board, size=512),
            write_to=render
        )
        render.seek(0)
        self.board_rendered = pygame.image.load(render)

    def __del__(self):
        pass # maybe delete extra file or smthn idk

    def update(self):
        self.screen.fill((0,0,0))
        board_update = False

        # check for close
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        # process cmd queue
        while not self.q.empty():
            cmd_tokens = self.q.get().data.split()
            if cmd_tokens[0] == 'stop':
                pygame.quit()
                sys.exit()

            elif cmd_tokens[0] == 'push':
                try:
                    for move in cmd_tokens[1:]:
                        self.board.push(move)
                except:
                    pass
                board_update = True

            elif cmd_tokens[0] == 'pop':
                depth = int(cmd_tokens[1])
                for _ in range(min(depth, len(self.board.move_stack))):
                    self.board.pop()
                board_update = True
                
        if board_update:
            self._render_board()
            # add stuff more move stack

        self.screen.blit(self.board_rendered, (0,0))
        # also render move stack
        pygame.display.update()

def main(args=None):
    rclpy.init(args=args)

    node = DebugDisplay()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()