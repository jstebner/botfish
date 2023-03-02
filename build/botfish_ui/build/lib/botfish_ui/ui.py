import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import Queue

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # why couldnt this just be an arg bro
import pygame
from pygame.locals import *
import time

SD = 2 # scaledown # TODO: change to 1 for final

# UTILS
pygame.init() # initing here cuz i need for global font
DIM = (1920//SD, 1080//SD)
EXPAND = 1.14 # make btn go big by 16%
EXPAND = 1 + (EXPAND-1)/SD
SMOLFONT = pygame.font.SysFont('monospace', 32//SD, bold=True)
BEEGFONT = pygame.font.SysFont('monospace', 100//SD, bold=True)
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
        self.times = (600, 600) # 0: white, 1: black
        # lord forgive me for what im about to do
        self.windows = {
            'start': {
                'func': self.start_screen,
                'text': {
                    'title': [
                        'Botfish',
                        'white',
                        (100,95),
                        BEEGFONT,
                    ],
                    'config': [
                        'config:',
                        'white',
                        (1100, 150),
                        SMOLFONT
                    ]
                },
                'rect': {
            
                },
                'btns': {
                    'white': {
                        'params' : [
                            'Bot Plays White',  # display text
                            (1100, 250),        # position
                            (360, 100),         # dimension
                            'white'             # toggled color
                        ],
                        'group': 'color'        # group assignment
                        # 'toggle': False
                        # 'curr_scale': 1.0
                    },
                    'black': {
                        'params': [
                            'Bot Plays Black',
                            (1460, 250),
                            (360, 100),
                            'white'
                        ],
                        'group': 'color'
                    },
                    
                    'left': {
                        'params': [
                            'Bot is Left',
                            (1100, 400),
                            (360, 100),
                            'white'
                        ],
                        'group': 'bot_side'
                    },
                    'right': {
                        'params': [
                            'Bot is Right',
                            (1460, 400),
                            (360, 100),
                            'white'
                        ],
                        'group': 'bot_side'
                    },
                    
                    'easy_diff': {
                        'params' : [
                            'ez',
                            (1100, 550),
                            (240,100),
                            'green'
                        ],
                        'group': 'difficulty'
                    },
                    'medi_diff': {
                        'params' : [
                            'mid',
                            (1340, 550),
                            (240,100),
                            'yellow'
                        ],
                        'group': 'difficulty'
                    },
                    'hard_diff': {
                        'params' : [
                            'uhoh',
                            (1580, 550),
                            (240, 100),
                            'red'
                        ],
                        'group': 'difficulty'
                    },

                    'start_btn': {
                        'params': [
                            '< Begin >',
                            (1100, 750), #(200->X, 800)
                            (720,100),
                            'white'
                        ],
                        'group': None
                    }
                }
            },
            'play': {
                'func': self.play_screen,
                'text': {
            
                },
                'rect': {
                    'bigboy': {
                        'params': [
                            'white',
                            (1094.4, 0), # TODO: make this change or smthn
                            (825.6, 1080),
                        ],

                    }
                },
                'btns': {
            
                }
            },
            'end': {
                'func': self.end_screen,
                'text': {
            
                },
                'rect': {
            
                },
                'btns': {

                },
            }
        }
        # take "input" data from above make it good ig idk
        for window_id in self.windows:
            # add button groups
            self.windows[window_id]['grps'] = dict()
            for btn_id in self.windows[window_id]['btns']:
                self.windows[window_id]['btns'][btn_id]['toggle'] = False # assign default state
                self.windows[window_id]['btns'][btn_id]['curr_scale'] = 1.0 # use for scale interp
                # assign button group
                group_id = self.windows[window_id]['btns'][btn_id]['group']
                if group_id is None:
                    continue
                if group_id not in self.windows[window_id]['grps']:
                    self.windows[window_id]['grps'][group_id] = [btn_id]
                    self.windows[window_id]['btns'][btn_id]['toggle'] = True
                else:
                    self.windows[window_id]['grps'][group_id].append(btn_id)
            for rect_id in self.windows[window_id]['rect']:
                self.windows[window_id]['rect'][rect_id]['curr_scale'] = 1.0 # use for scale interp
        
        self.ui_ping_pub = self.create_publisher(
            String,
            'ui_ping',
            10
        )
        self.ui_msg_pub = self.create_publisher(
            String,
            'ui_msg',
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

    def _draw_text(self, text, color, pos, font):
        text_obj = font.render(text, True, color)
        text_rect = text_obj.get_rect()
        text_rect.topleft = scaler(*pos)
        self.screen.blit(text_obj, text_rect)
        
    def _draw_rect(self, color, pos, dims):
        rectangle = pygame.Rect(*scaler(*pos), *scaler(*dims))
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
        # TODO: make this not shitty
        if btn.collidepoint(mpos): # hover
            if self.windows[screen_id]['btns'][btn_id]['curr_scale'] < EXPAND:
                self.windows[screen_id]['btns'][btn_id]['curr_scale'] += 0.02
            interp_exp = self.windows[screen_id]['btns'][btn_id]['curr_scale']
            
            pos[0] -= int(dim[0]*(interp_exp-1)/2)
            pos[1] -= int(dim[1]*(interp_exp-1)/2)
            dim[0] *= interp_exp
            dim[1] *= interp_exp
            btn = pygame.Rect(*pos, *dim)
            self.foreground = ((b_color, btn), (text, t_color, unscaler(*text_pos)))
            
            # bro what is this
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
            if self.windows[screen_id]['btns'][btn_id]['curr_scale'] > 1:
                self.windows[screen_id]['btns'][btn_id]['curr_scale'] -= 0.02
            interp_exp = self.windows[screen_id]['btns'][btn_id]['curr_scale']
            pos[0] -= int(dim[0]*(interp_exp-1)/2)
            pos[1] -= int(dim[1]*(interp_exp-1)/2)
            dim[0] *= interp_exp
            dim[1] *= interp_exp
            btn = pygame.Rect(*pos, *dim)
            pygame.draw.rect(self.screen, b_color, btn)
            self._draw_text(text, t_color, unscaler(*text_pos), SMOLFONT)
        
        self.windows[screen_id]['btns'][btn_id]['toggle'] = toggle

    def _draw_content(self, screen_id: str, mpos: tuple, clicking: bool):
        for txt_params in self.windows[screen_id]['text'].values():
            self._draw_text(*txt_params)
        
        for rect_data in self.windows[screen_id]['rect'].values():
            self._draw_rect(*rect_data['params'])
        
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
                if event.key == K_ESCAPE: # TODO: maybe make this esc menu
                    close()
                if event.key == K_SPACE:
                    if self.is_player_turn and self.curr_screen == 'play':
                        msg = String()
                        msg.data = 'PING'
                        self.ui_msg_pub.publish(msg)
                        self.is_player_turn = False
        return (mx, my), clicking
    
    def update(self,): # screen "skeleton"
        self.screen.fill(CLRS['black'])
        mouse_pos, clicking = self.event_listener()
        self.windows[self.curr_screen]['func'](mouse_pos, clicking)
        pygame.display.update()

    def start_screen(self, mpos, clicking): # setup params n whatnot
        self._draw_content('start', mpos, clicking)
        if self.windows['start']['btns']['start_btn']['toggle']:
            args = {}
            for btn_id, data in self.windows['start']['btns'].items():
                if btn_id == 'start_btn':
                    continue
                if data['group'] is None:
                    args[btn_id] = data['toggle']
                elif data['toggle']:
                    args[data['group']] = btn_id
            
            if args['color'] == 'white':
                wlbr = args['bot_side'] == 'left'
            else:
                wlbr = args['bot_side'] == 'right'
            del args['bot_side']
            if wlbr:
                self.windows['play']['rect']['bigboy']['params'][1] = (0,0) # move box to left
            
            msg = String()
            msg.data = ' '.join(f'{key}={val}' for key, val in args.items())
            self.ui_msg_pub.publish(msg)
            self.curr_screen = 'play'
    
    def play_screen(self, mpos, clicking): # timer
        # TODO: the white half of the timer i think         
        self._draw_content('play', mpos, clicking)
        
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