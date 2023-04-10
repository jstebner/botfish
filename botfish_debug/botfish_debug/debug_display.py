import os
import sys
import io

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64
from multiprocessing import Queue

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
from pygame.locals import *
import chess
import chess.svg
import cairosvg

SIZE = (512 + 256,1024)
WIDGET_SIZE = 512
FONT_SIZE = 20
HISTORY = 20
PERIOD_s = 1/30 # 10 UpS

class DebugDisplay(Node):
    def __init__(self):
        pygame.display.set_caption('Debug')
        super().__init__('debug_display')
        
        self.cmd_q = Queue()
        self.cam_q = Queue()
        self.ui_q = Queue()
        self.screen = pygame.display.set_mode(SIZE) # testing
        self.FONT = pygame.font.SysFont('monospace', FONT_SIZE)
        self.ping_count = 0 # used to identify pings received from ui
        
        self.board = chess.Board()
        self.board_rendered = None
        self._render_board()
        self.camera_viewer = None
        
        self.user_text = str()
        
        self.nq = None
        
        self.debug_cmd_pub = self.create_publisher(
            String,
            'debug_cmd',
            10
        )
        self.player_move_pub = self.create_publisher(
            String,
            'player_move',
            10
        )
        self.perform_nqueens_pub = self.create_publisher(
            Int64,
            'perform_nqueens',
            10
        )
        self.debug_cmd_sub = self.create_subscription(
            String,
            'debug_cmd',
            self.cmd_q.put,
            10
        )
        self.camera_feed_sub = self.create_subscription(
            String, # TODO: this should be image
            'camera_feed',
            self.cam_q.put,
            10
        )
        self.ui_msg_sub = self.create_subscription(
            String,
            'ui_msg',
            self.ui_q.put,
            10
        )
        self.player_move_sub = self.create_subscription(
            String,
            'player_move',
            self.cmd_q.put,
            10
        )
        self.bot_move_sub = self.create_subscription(
            String,
            'bot_move',
            self.cmd_q.put,
            10
        )
        self.tiles_sub = self.create_subscription(
            String,
            'tiles',
            self.process_nq,
            10
        )
        self.timer = self.create_timer(
            PERIOD_s,
            self.update
        )
    
    def _render_board(self, board=None):
        if board is None:
            board = self.board
        
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
    
    def process_nq(self, msg):
        tiles = msg.data
        print(msg.data)
        tiles = [tiles[2*i:2*i+2] for i in range(len(tiles)//2)]
        tiles = list(map(lambda c:((c[0]), int(c[1])), tiles))
        

    def parse_cmd(self, inp):
        msg = String()

        cmd_tokens = inp.lower().split()
        if not cmd_tokens:
            return
        
        # cmds that need parsing
        elif cmd_tokens[0] == 'help':
            # stop, stopall, push (uci), pop (int), help, switch, emote (key), svn
            print('you got this lil bro :)')

        elif cmd_tokens[0] in ['stop','stopall']:
            msg.data = cmd_tokens[0]
            self.debug_cmd_pub.publish(msg)
            sys.exit()
        
        elif cmd_tokens[0] == 'svn': # spoof vision node
            if len(cmd_tokens) == 1:
                print('need move to spoof')
                return
            if len(cmd_tokens) > 2:
                print('too many args')
                return
            if not self._uci_check(cmd_tokens[1]):
                print('move is invalid')
                return
            
            msg.data = f'push {cmd_tokens[1]}'
            self.player_move_pub.publish(msg)
            return

        elif cmd_tokens[0] == 'nq':
            msg = Int64()
            if len(cmd_tokens) == 1:
                n = 4
            elif len(cmd_tokens) > 2:
                print('too many arg')
                return
            elif not cmd_tokens[1].isdigit():
                print('need a number')
                return
            else:
                n = int(cmd_tokens[1])
                
            msg.data = n
            self.perform_nqueens_pub.publish(msg)
            return

        elif cmd_tokens[0] == 'push':
            if len(cmd_tokens) == 1:
                print('need a move to push bozo')
                return
            ok = True
            for move in cmd_tokens[1:]:
                if not self._uci_check(move):
                    ok = False
                    print(f'{move} is invalid')
            if not ok:
                return
        
        elif cmd_tokens[0] == 'pop':
            if len(cmd_tokens) > 2:
                print('too many jawns bruv')
                return
            elif len(cmd_tokens) == 1:
                cmd_tokens.append('1')
            elif not cmd_tokens[1].isdigit():
                print('gimme a num idot')
                return
        
        elif cmd_tokens[0] == 'emote':
            if len(cmd_tokens) == 1:
                print('need which emote man')
                return
            if len(cmd_tokens) > 2:
                print('too many args')
                return
            if cmd_tokens[1] == '--list':
                print('available emotes:\n -wave\n -calibrate') # TODO: update as necessary
                return
            if cmd_tokens[1] not in [ # TODO: update as necessary
                'wave',
                'calibrate'
            ]:
                print('invalid emote')
                return

        else:
            print('huh')
        
        msg.data = ' '.join(cmd_tokens)
        self.debug_cmd_pub.publish(msg)
    
    def event_listener(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_RETURN:
                    self.parse_cmd(self.user_text)
                    self.user_text = ''
                elif event.key == K_BACKSPACE:
                    if len(self.user_text):
                        self.user_text = self.user_text[:-1]
                else:
                    self.user_text += event.unicode
    
    def update(self):
        # TODO: add supplementals to board like arrows n stuff
        self.screen.fill((0,0,0))
        board_update = False

        # check for close
        self.event_listener()

        # process cmd queue
        while not self.cmd_q.empty():
            cmd_tokens = self.cmd_q.get().data.split()
            if cmd_tokens[0] in ['stop', 'stopall']:
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
            self.camera_viewer = self.cam_q.get() # dump queue for now
        
        # draw board
        if board_update:
            self._render_board()
        self.screen.blit(self.board_rendered, (0,0))
        
        # TODO: draw text input
        # GO HERE >>> https://www.geeksforgeeks.org/how-to-create-a-text-input-box-with-pygame/
        self._draw_text(
            self.user_text,
            x = 512,
            y = 0
        )
        
        # draw move stack
        last = max(0, len(self.board.move_stack) - HISTORY)
        for pos, idx in enumerate(range(last, len(self.board.move_stack))):
            self._draw_text(
                text = f'{str(idx).rjust(3)}: [{"WHITE" if idx%2==0 else "BLACK"}] {self.board.move_stack[idx]}', # we do a lil string formatting
                x = 512, 
                y = 20 + pos*FONT_SIZE + 10
            )

        # TODO: make this do more than just get pings
        # process ui queue
        while not self.ui_q.empty():
            msg = self.ui_q.get().data
            if msg == 'ping':
                self.ping_count += 1

        # draw ping count
        self._draw_text(
            text = f'pings received: {self.ping_count}',
            x = 512,
            y = 995
        )

        # TODO: make this the actual camera
        # draw camera vision with extras
        if self.camera_viewer is None:
            pygame.draw.rect(self.screen, (100,100,100), pygame.Rect(0, 512, WIDGET_SIZE, WIDGET_SIZE))
            self._draw_text(
                text = 'Camera Disconnected',
                x = 160,
                y = 745
            )
        else:
            pass # idk bro figure it out
        
        pygame.display.update()

def main(args=None):
    pygame.init()
    rclpy.init(args=args)

    node = DebugDisplay()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()