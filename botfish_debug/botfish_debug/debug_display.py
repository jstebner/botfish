import sys
import os
import io

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # TODO: also import msg for images

import chess
import chess.svg
import pygame
import cairosvg
from pygame.locals import *
from multiprocessing import Queue

SIZE = (512 + 256,1024)
WIDGET_SIZE = 512
FONT_SIZE = 20
HISTORY = 20

class DebugDisplay(Node):
    PERIOD_s = 1/10 # 10 UpS
    DIRPATH = os.path.dirname(__file__)

    def __init__(self):
        pygame.init()
        pygame.display.set_caption('Debug')
        super().__init__('debug_display')
        
        self.cmd_q = Queue()
        self.cam_q = Queue()
        self.screen = pygame.display.set_mode(SIZE) # testing
        self.FONT = pygame.font.SysFont('monospace', FONT_SIZE)
        
        self.board = chess.Board()
        self.board_rendered = None
        self._render_board()
        self.camera_viewer = None
        
        self.update_debug_board_sub = self.create_subscription(
            String,
            'update_debug_board',
            self.cmd_q.put,
            10
        )
        self.camera_debug_sub = self.create_subscription(
            String, # TODO: this should be image
            'camera_debug',
            self.cam_q.put,
            10
        )
        self.timer = self.create_timer(
            self.PERIOD_s,
            self.update
        )
    
    def _render_board(self):
        render = io.BytesIO()
        cairosvg.svg2png(
            chess.svg.board(self.board, size=WIDGET_SIZE),
            write_to=render
        )
        render.seek(0)
        self.board_rendered = pygame.image.load(render)
    
    def _draw_text(self, text, x, y):
        text_obj = self.FONT.render(text, True, (255,255,255))
        text_rect = text_obj.get_rect()
        text_rect.topleft = (x, y)
        self.screen.blit(text_obj, text_rect)

    def update(self):
        self.screen.fill((0,0,0))
        board_update = False

        # check for close
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        # process cmd queue
        while not self.cmd_q.empty():
            cmd_tokens = self.cmd_q.get().data.split()
            if cmd_tokens[0] == 'stop':
                pygame.quit()
                sys.exit()

            elif cmd_tokens[0] == 'push':
                try:
                    for move in cmd_tokens[1:]:
                        self.board.push(chess.Move.from_uci(move))
                except:
                    pass
                board_update = True

            elif cmd_tokens[0] == 'pop':
                depth = int(cmd_tokens[1])
                for _ in range(min(depth, len(self.board.move_stack))):
                    self.board.pop()
                board_update = True
                
        # process cam queue
        while not self.cam_q.empty():
            self.camera_viewer = self.cam_q.get() # dump queue
        
        # draw board
        if board_update:
            self._render_board()
        self.screen.blit(self.board_rendered, (0,0))
        
        # draw move stack
        last = max(0, len(self.board.move_stack) - HISTORY)
        for pos, idx in enumerate(range(last, len(self.board.move_stack))):
            self._draw_text(
                text = f'{str(idx).rjust(3)}: [{"WHITE" if idx%2==0 else "BLACK"}] {self.board.move_stack[idx]}', # we do a lil string formatting
                x = 512, 
                y = pos*FONT_SIZE + 5
            )

        # draw camera vision with extras
        if self.camera_viewer is None:
            pygame.draw.rect(self.screen, (100,100,100), pygame.Rect(0, 512, WIDGET_SIZE, WIDGET_SIZE))
            
            self._draw_text(
                text = 'Camera Disconnected',
                x = 200,
                y = 750
            )
        
        pygame.display.update()

def main(args=None):
    rclpy.init(args=args)

    node = DebugDisplay()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()