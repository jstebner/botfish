import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import chess
import chess.svg

import pygame
from pygame.locals import *

from multiprocessing import Queue

class DebugDisplay(Node):
    PERIOD_s = 1/10 # 10 UpS
    FONT = pygame.font.SysFont('Calibri', 35)
    PATH = os.path.dirname(__file__)

    def __init__(self):
        self.q = Queue()
        self.board = chess.Board()
        
        super().__init__('debug_display')
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
    
        pygame.init()
        self.screen = pygame.display.set_mode((512, 512)) # testing
        self.x = 0
        self.y = 30

    def __del__(self):
        print(chr(7))

    def update(self):
        self.screen.fill((0,0,0))
        self.x += 5
        self.x %= 1080
        box = pygame.Rect(self.x, self.y, 20, 20)
        pygame.draw.rect(self.screen,(64,31,255),box)

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
                    continue    

            elif cmd_tokens[0] == 'pop':
                depth = int(cmd_tokens[1])
                for _ in range(min(depth, len(self.board.move_stack))):
                    self.board.pop()
        
        # update visuals here (board and move stack)

        pygame.display.update()

def main(args=None):
    rclpy.init(args=args)

    node = DebugDisplay()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()