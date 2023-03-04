# note for the future: 
#   never let me do frontend
#       - jong

import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import Queue

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # why couldnt this just be an arg bro
import pygame
from pygame.locals import *
from time import time

SD = 2 # scaledown # TODO: change to 1 for final

# UTILS
pygame.init() # initing here cuz i need for global font
DIM = (1920//SD, 1080//SD)
EXPAND = 1.16 # make btn go big by 16%
SMOLFONT = pygame.font.SysFont('monospace', 32//SD, bold=True)
BEEGFONT = pygame.font.SysFont('monospace', 100//SD, bold=True)
CLRS = {
    'white':    (249,246,240),  # F9 F6 F0
    'black':    (23,21,21),     # 17 15 15
    'gray':     (160,158,164),  # A0 9E A4
    'red':      (219,31,72),    # DB 1F 48
    'green':    (184,238,48),   # B8 EE 30
    'yellow':   (249,208,48),   # F9 D0 30
}
def scaler(*args):
    return [arg//SD for arg in args]
def unscaler(*args): # you have no right to justdge me for this
    return [arg*SD for arg in args]

# LOGO = pygame.image.load(os.path.join(os.path.dirname(__file__), 'logo.png'))

class PriorQueue:
    def __init__(self):
        self.q = list()
    def push(self, key, val):
        if key == -1:
            self.q.append((key, val))
        for i, item in enumerate(self.q):
            if item[0] > key:
                self.q.insert(i, (key, val))
                break
    def pop(self):
        item = self.q.pop(0)
        return item[1]
    def empty(self):
        return len(self.q) != 0

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
        self.is_player_turn = True
        self.foreground = None
        self.play_bg_clr = 'black'
        self.is_left = True
        self.paused = False
        self.chess_timers = [
            [None, 600], # prev_ts, total_t
            [None, 600],
        ]
        self.curr_screen = 'start'
        # lord forgive me for what im about to do
        self.windows = {
            'start': {
                'func': self.start_screen,
                'text': {
                    'title': [
                        'botfish :)',
                        'white',
                        (100,95),
                        BEEGFONT,
                    ],
                    'config': [
                        'config:',
                        'white',
                        (1100, 100),
                        SMOLFONT
                    ]
                },
                'rect': {
            
                },
                'btns': {
                    'white': {
                        'params': [
                            'bot plays white',  # display text
                            (1100, 200),        # position
                            (360, 100),         # dimension
                            'white'             # toggled color
                        ],
                        'group': 'color'        # group assignment
                        # 'toggle': False
                        # 'curr_scale': 1.0
                    },
                    'black': {
                        'params': [
                            'bot plays black',
                            (1460, 200),
                            (360, 100),
                            'white'
                        ],
                        'group': 'color'
                    },
                    
                    'left': {
                        'params': [
                            'bot is left',
                            (1100, 350),
                            (360, 100),
                            'white'
                        ],
                        'group': 'bot_side'
                    },
                    'right': {
                        'params': [
                            'bot is right',
                            (1460, 350),
                            (360, 100),
                            'white'
                        ],
                        'group': 'bot_side'
                    },
                    
                    'easy_diff': {
                        'params' : [
                            'ez',
                            (1100, 500),
                            (240,100),
                            'green'
                        ],
                        'group': 'difficulty'
                    },
                    'medi_diff': {
                        'params' : [
                            'mid',
                            (1340, 500),
                            (240,100),
                            'yellow'
                        ],
                        'group': 'difficulty'
                    },
                    'hard_diff': {
                        'params' : [
                            'uhoh',
                            (1580, 500),
                            (240, 100),
                            'red'
                        ],
                        'group': 'difficulty'
                    },
                    'start_btn': {
                        'params': [
                            '- begin -',
                            (1100, 650), #(200->X, 800)
                            (720,100),
                            'white'
                        ],
                        'group': None
                    },
                    'quit': {
                        'params': [
                            'quit',
                            (1460, 800),
                            (360, 100),
                            'red'
                        ],
                        'group': None
                    }
                }
            },
            'play': {
                'func': self.play_screen,
                'text': {
                    'left_time': [
                        '10:00.00',
                        'black',
                        (0,0),
                        BEEGFONT
                    ],
                    'right_time': [
                        '10:00.00',
                        'white',
                        (960,0),
                        BEEGFONT
                    ]
                },
                'rect': {
                    'bigboy': {
                        'params': [
                            'white', # clr duh
                            (0, 0), # pos
                            (960, 1080), # dim
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
            },
            'pause': {
                'func': self.pause_screen,
                'text': {
                    'title': [
                        'paused',
                        'white',
                        (100,95), # TODOL amke this good
                        BEEGFONT
                    ]
                },
                'rect': {
            
                },
                'btns': {
                    'unpause': {
                        'params': [
                            'unpause',
                            (100, 300),
                            (250, 100),
                            'white'
                        ],
                        'group': None
                    },
                    'home': {
                        'params': [
                            'go home',
                            (100, 450),
                            (250, 100),
                            'white'
                        ],
                        'group': None
                    },
                    'quit': {
                        'params': [
                            'quit app',
                            (100, 600),
                            (250, 100),
                            'red'
                        ],
                        'group': None
                    },
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
    
    # TODO: make this better or smthn idk
    def close(self):
        msg = String()
        msg.data = 'stopall'
        self.ui_msg_pub.publish(msg)

        pygame.quit()
        exit()

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
        # TODO: also make draw order queue with prior by curr_scale
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
        # TODO: rewrite this to render objects, put on pq, then draw
        for txt_params in self.windows[screen_id]['text'].values():
            self._draw_text(*txt_params)
        
        self.foreground = None
        for btn_id in self.windows[screen_id]['btns']:
            self._draw_button(screen_id, btn_id, mpos, clicking)
            
        if self.foreground:
            pygame.draw.rect(self.screen, *(self.foreground[0]))
            self._draw_text(*(self.foreground[1]), SMOLFONT)

    # just dont ask
    def switch(self):
        self.is_left ^= True # dont ask
        self.is_player_turn ^= True # dont ask
        self.chess_timers[self.is_left][0] = None # you can ask abt this one tho
        self.chess_timers[not self.is_left][0] = time() # dont ask        

    def event_listener(self):
        # TODO: make
        while not self.debug_q.empty():
            cmd_tokens = self.debug_q.get().data.split()
            if cmd_tokens[0] == 'stopall':
                self.close()
            
            elif cmd_tokens[0] == 'switch':
                self.switch()
        
        # TODO: make this
        while not self.gamestate_q.empty():
            tokens = self.gamestate_q.get().data.split()

            
        mx, my = pygame.mouse.get_pos()
        clicking = False
        for event in pygame.event.get():
            if event.type == QUIT:
                self.close()
            if event.type == MOUSEBUTTONDOWN:
                clicking = True
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE and self.curr_screen in ['play', 'pause']:
                    if self.paused: # unpausing
                        self.chess_timers[not self.is_left][0] = time()
                    self.paused ^= True # dont ask
                if event.key == K_SPACE:
                    if self.is_player_turn and self.curr_screen == 'play':
                        msg = String()
                        msg.data = 'ping'
                        self.ui_msg_pub.publish(msg)
                        self.switch()
        return (mx, my), clicking
    
    def update(self,): # screen "skeleton"
        self.screen.fill(CLRS[self.play_bg_clr if not self.paused else 'black'])
        mouse_pos, clicking = self.event_listener()
        self.windows[self.curr_screen if not self.paused else 'pause']['func'](mouse_pos, clicking)
        pygame.display.update()

    def start_screen(self, mpos, clicking): # setup params n whatnot
        if self.windows['start']['btns']['quit']['toggle']:
            self.close()

        # self.screen.blit(LOGO, (100,100))
        
        if not self.windows['start']['btns']['start_btn']['toggle']:
            self._draw_content('start', mpos, clicking)
            return
        
        # end of screen
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
        del args['quit']
        if not wlbr:
            self.windows['play']['rect']['bigboy']['params'][0] = 'black'
            self.windows['play']['text']['left_time'][1] = 'white'
            self.windows['play']['text']['right_time'][1] = 'black'
            self.play_bg_clr = 'white'
            self.is_left = False
        else:
            self.windows['play']['rect']['bigboy']['params'][0] = 'white'
            self.windows['play']['text']['left_time'][1] = 'black'
            self.windows['play']['text']['right_time'][1] = 'white'
            self.play_bg_clr = 'black'
            self.is_left = True
        self.chess_timers[not self.is_left][0] = time() # dont ask
        self.chess_timers[0][1] = 600
        self.chess_timers[1][1] = 600
        self.windows['play']['text']['left_time'][0] = '10:00.00'
        self.windows['play']['text']['right_time'][0] = '10:00.00'
        
        msg = String()
        msg.data = ' '.join(f'{key}={val}' for key, val in args.items())
        print(msg.data)
        self.ui_msg_pub.publish(msg)
        self.curr_screen = 'play'
    
    def play_screen(self, mpos, clicking): # timer
        # was gonna add this to the framework above but i dont wanna and this will literally only every be used once
        bigboy = self.windows['play']['rect']['bigboy']
        if self.is_left and bigboy['curr_scale'] < EXPAND:
            bigboy['curr_scale'] += 0.04
        if not self.is_left and bigboy['curr_scale'] > 2 - EXPAND:
            bigboy['curr_scale'] -= 0.04

        # TODO: timers be actin funny when they go negative but like do i really need to fix that
        # avert your eyes
        self.windows['play']['text']['left_time'][2] = (
            (bigboy['params'][2][0] * bigboy['curr_scale'] - unscaler(BEEGFONT.size(self.windows['play']['text']['left_time'][0])[0])[0]) / 2,
            (1080 - unscaler(BEEGFONT.size(self.windows['play']['text']['left_time'][0])[1])[0]) / 2
        )
        self.windows['play']['text']['right_time'][2] = (
            960 + (bigboy['params'][2][0] * bigboy['curr_scale'] - unscaler(BEEGFONT.size(self.windows['play']['text']['right_time'][0])[0])[0]) / 2,
            (1080 - unscaler(BEEGFONT.size(self.windows['play']['text']['right_time'][0])[1])[0]) / 2
        )
        
        self.chess_timers[not self.is_left][1] -= time() - self.chess_timers[not self.is_left][0]
        self.chess_timers[not self.is_left][0] = time()
        self.windows['play']['text'][f'{"left" if self.is_left else "right"}_time'][0] = '{0:02.0f}:{1:05.2f}'.format(*divmod(self.chess_timers[not self.is_left][1], 60)) # please dont ask
        
        clr, pos, dim = bigboy['params']
        dim = list(dim)
        dim[0] = dim[0]*bigboy['curr_scale'] # only scale horizontally
        self._draw_rect(clr, pos, dim)
        self._draw_text(*self.windows['play']['text']['left_time'])
        self._draw_text(*self.windows['play']['text']['right_time'])
        
    def pause_screen(self, mpos, clicking):
        if self.windows['pause']['btns']['quit']['toggle']:
            self.close()

        if self.windows['pause']['btns']['unpause']['toggle']:
            self.windows['pause']['btns']['unpause']['toggle'] = False
            self.paused = False

        if self.windows['pause']['btns']['home']['toggle']:
            self.windows['pause']['btns']['home']['toggle'] = False
            self.windows['start']['btns']['start_btn']['toggle'] = False
            self.curr_screen = 'start'
            self.play_bg_clr = 'black'
            self.paused = False

        self._draw_content('pause', mpos, clicking)

    def end_screen(self, mpos, clicking): # gameover / cleanup n whatnot
        # TODO: this
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
