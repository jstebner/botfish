import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import Queue

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # why couldnt this just be an arg bro
import pygame
from pygame.locals import *

SD = 2 # scaledown # TODO: change to 1 for final

# UTILS
pygame.init() # initing here cuz i need for global font
DIM = (1920//SD, 1080//SD)
EXPAND = 1.16 # make btn go big by 10%
EXPAND = 1 + (EXPAND-1)/SD
SMOLFONT = pygame.font.SysFont('monospace', 30//SD, bold=True)
BEEGFONT = pygame.font.SysFont('monospace', 60//SD, bold=True)
CLRS = {
    'white':    (249,246,240),  # F9 F6 F0
    'black':    (23,21,21),     # 17 15 15
    'gray':     (160,158,164),  # A0 9E A4
    'red':      (219,31,72),    # db 1f 48
    'green':    (184,238,48),   # b8 ee 30
    'yellow':   (249,208,48),   # f9 d0 30
}
def scaler(*args):
    return [arg//SD for arg in args]
def unscaler(*args): # you have no right to justdge me for this
    return [arg*SD for arg in args]

# TODO: make this better or smthn idk
def close():
    pygame.quit()
    sys.exit()

PERIOD_s = 1/40 # 40 UpS
class UIController(Node):
    def __init__(self):
        # pygame.init()
        super().__init__('ui_controller')
        self.screen = pygame.display.set_mode(DIM, pygame.NOFRAME) # final
        self.screen = pygame.display.set_mode(DIM,) # testing
        pygame.display.set_caption('Botfish Chess Timer')

        self.debug_q = Queue()
        self.gamestate_q = Queue()
        self.is_player_turn = True # TODO: make this depend on the game or whatev
        self.foreground = None
        self.curr_screen = 'start'
        # lord forgive me for what im about to do
        self.windows = {
            'start': {
                'func': self.start_screen,
                'btns': {
                    'easy_diff': {
                        'params' : [
                            'ez',       # display text
                            (150, 50),  # position
                            (200,100),  # dimension
                            'green',    # toggled color
                        ],
                        'group': 'difficulty', # group assignment
                    },
                    'medi_diff': {
                        'params' : [
                            'mid',
                            (350, 50),
                            (200,100),
                            'yellow',
                        ],
                        'group': 'difficulty',
                    },
                    'hard_diff': {
                        'params' : [
                            'uhoh',
                            (550, 50),
                            (200,100),
                            'red'
                        ],
                        'group': 'difficulty',
                    },
                }
            },
            'play': {
                'func': self.play_screen,
                'btns': {

                }
            },
            'end': {
                'func': self.end_screen,
                'btns': {

                }
            }
        }
        # take "input" data from above make it good ig idk
        for window_id in self.windows:
            # add button groups
            self.windows[window_id]['grps'] = dict()
            for btn_id in self.windows[window_id]['btns']:
                # assign button group
                self.windows[window_id]['btns'][btn_id]['toggle'] = False # assign default state
                group_id = self.windows[window_id]['btns'][btn_id]['group']
                if group_id is None:
                    continue
                if group_id not in self.windows[window_id]['grps']:
                    self.windows[window_id]['grps'][group_id] = [btn_id]
                    self.windows[window_id]['btns'][btn_id]['toggle'] = True
                else:
                    self.windows[window_id]['grps'][group_id].append(btn_id)
        
        self.ui_ping_pub = self.create_publisher(
            String,
            'ui_ping',
            10
        )
        self.ui_config_pub = self.create_publisher(
            String,
            'ui_config',
            10
        )
        self.gamestate_sub = self.create_subscription(
            String,
            'gamestate',
            self.gamestate_q.put,
            10
        )
        self.debug_cmd_sub = self.create_subscription(
            String,
            'debug_cmd',
            self.debug_q.put, # puts received msg in q directly
            10
        )
        self.timer = self.create_timer(
            PERIOD_s,
            self.update # idk if this is scuffed but whatev
        )

    def _draw_text(self, text, color, x, y, font):
        text_obj = font.render(text, True, color)
        text_rect = text_obj.get_rect()
        text_rect.topleft = scaler(x, y)
        self.screen.blit(text_obj, text_rect)
        
    def _draw_rect(self, color, x, y, dims):
        rectangle = pygame.Rect(scaler(x), scaler(y), *scaler(*dims))
        pygame.draw.rect(self.screen, color, rectangle)

    # nightmare
    def _draw_button(self, screen_id: str, btn_id: str, mpos: tuple, click: bool) -> bool:
        text, pos, dim, color = self.windows[screen_id]['btns'][btn_id]['params']
        toggle = self.windows[screen_id]['btns'][btn_id]['toggle']
        group = self.windows[screen_id]['btns'][btn_id]['group']

        pos = scaler(*pos) # removes reference 
        dim = scaler(*dim)
        text_pos = (
            int(pos[0] + dim[0]/2 - SMOLFONT.size(text)[0]/2),
            int(pos[1] + dim[1]/2 - SMOLFONT.size(text)[1]/2)
        )

        if toggle:
            b_color = CLRS[color]
            t_color = CLRS['black']
        else:
            b_color = CLRS['gray']
            t_color = CLRS['white']

        btn = pygame.Rect(*pos, *dim)
        if btn.collidepoint(mpos): # hover
            pos[0] -= int(dim[0]*(EXPAND-1)/2)
            pos[1] -= int(dim[1]*(EXPAND-1)/2)
            dim[0] *= EXPAND
            dim[1] *= EXPAND
            btn = pygame.Rect(*pos, *dim)
            self.foreground = ((b_color, btn), (text, t_color, *unscaler(*text_pos)))
            
            if click:
                if group:
                    if not toggle:
                        toggle = True
                        for anti_btn_id in self.windows[screen_id]['grps'][group]:
                            if btn_id == anti_btn_id:
                                continue
                            self.windows[screen_id]['btns'][anti_btn_id]['toggle'] = False
                else:
                    toggle ^= True # dont ask
        else:
            pygame.draw.rect(self.screen, b_color, btn)
            self._draw_text(text, t_color, *unscaler(*text_pos), SMOLFONT)
        
        self.windows[screen_id]['btns'][btn_id]['toggle'] = toggle

    def _draw_buttons(self, screen_id: str, mpos: tuple, clicking: bool):
        self.foreground = None

        for btn_id in self.windows[screen_id]['btns']:
            self._draw_button(screen_id, btn_id, mpos, clicking)
            
        if self.foreground:
            pygame.draw.rect(self.screen, *(self.foreground[0]))
            self._draw_text(*(self.foreground[1]), SMOLFONT)
        

    def event_listener(self):
        mx, my = pygame.mouse.get_pos()
        clicking = False
        for event in pygame.event.get():
            if event.type == QUIT:
                close()
            if event.type == MOUSEBUTTONDOWN:
                clicking = True
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    close()
                if event.key == K_SPACE:
                    if self.is_player_turn:
                        ping = String()
                        ping.data = f'ping'
                        self.ui_ping_pub.publish(ping)
                        self.is_player_turn = False
        return (mx, my), clicking
    
    def update(self,): # screen "skeleton"
        self.screen.fill(CLRS['black'])
        mouse_pos, clicking = self.event_listener()
        self.windows[self.curr_screen]['func'](mouse_pos, clicking)
        pygame.display.update()

    def start_screen(self, mpos, clicking): # setup params n whatnot
        self._draw_buttons('start', mpos, clicking)
    
    
    def play_screen(self, mpos, clicking): # timer
        # TODO: the white half of the timer i think         
        self._draw_rect(
            screen=self.screen,
            color=CLRS['white'],
            x=0, y=0,
            dims=(DIM[0]//2, DIM[1])
        )
        
        # TODO: mfin timers n whatnot
        
        # process debug queue
        while not self.debug_q.empty():
            cmd_tokens = self.debug_q.get().data.split()
            if cmd_tokens[0] == 'stop':
                close()
            
            elif cmd_tokens[0] == 'switch':
                self.is_player_turn ^= True # dont ask
        

    def end_screen(self, mpos, clicking): # gameover / cleanup n whatnot
        pass


def main(args=None):
    rclpy.init(args=args)

    ui = UIController()
    rclpy.spin(ui)
    # TODO: figure out how tf to get out of a node the right way

    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
