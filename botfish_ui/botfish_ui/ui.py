import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multiprocessing import Queue

import pygame
from pygame.locals import *

pygame.init()
# UTILS
DIM = (1920//2, 1080//2) # rmv //2 for final
PERIOD_s = 1/32 # 32 UpS
EXPAND = 1.05 # make btn go big by 5%
SMOLFONT = pygame.font.SysFont('monospace', 10)
BEEGFONT = pygame.font.SysFont('monospace', 30)
CLRS = { # TODO: make these better and more
    'white': (255,255,255),
    'black': (0,0,0),
    'gray50': (50,50,50),
    'gray150': (150,150,150),
    'gray200': (200,200,200)
}

def draw_text(screen, text, color, x, y, is_big = False):
    font = BEEGFONT if is_big else SMOLFONT
    text_obj = font.render(text, True, color)
    text_rect = text_obj.get_rect()
    text_rect.topleft = (x, y)
    screen.blit(text_obj, text_rect)
    
def draw_rect(screen, color, x, y, dims):
    rectangle = pygame.Rect(x, y, *dims)
    pygame.draw.rect(screen, color, rectangle)

def draw_button(screen: pygame.display, text: str, pos: tuple, dim: tuple, toggle: bool, mpos: tuple, click: bool) -> bool:
    smolfont_size = 10
    pos = list(pos) # removes reference 
    dim = list(dim)
    text_pos = (
        int(dim[0]/2 - len(text)*smolfont_size/2),
        int(dim[1]/2 - smolfont_size/2),
    )
    btn = pygame.Rect(*pos, *dim)
    if btn.collidepoint(mpos): # hover
        pos[0] -= int(dim[0]*(1-EXPAND))
        pos[1] -= int(dim[1]*(1-EXPAND))
        dim[0] *= EXPAND
        dim[1] *= EXPAND
        btn = pygame.Rect(*pos, *dim)
        
        if click:
            toggle ^= True # dont ask
    if toggle:
        b_color = CLRS['white']
        t_color = CLRS['gray50']
    else:
        b_color = CLRS['gray50']
        t_color = CLRS['white']
    
    pygame.draw.rect(screen, b_color, btn)
    draw_text(screen, text, t_color, *text_pos)
    
    return toggle

class UIController(Node):
    def __init__(self):
        # pygame.init()
        super().__init__('ui_controller')
        # self.screen = pygame.display.set_mode(DIM, pygame.NOFRAME) # final
        self.screen = pygame.display.set_mode(DIM,) # testing
        pygame.display.set_caption('Botfish Chess Timer')

        self.debug_q = Queue()
        self.gamestate_q = Queue()
        self.is_player_turn = True # TODO: make this depend on the game or whatev
        self.curr_screen = 'start'
        self.screens = {
            'start': {
                'func': self.start_screen,
                'btns': {
                    'test': {
                        'toggle' : False,
                        'params' : ("test",(50, 50),(200,100))
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
        
        self.ui_ping_pub = self.create_publisher(
            String,
            'ui_ping',
            10
        )
        # TODO: add anotha pub guy for setup stuffs here
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

    def event_listener(self):
        mx, my = pygame.mouse.get_pos()
        clicking = False
        for event in pygame.event.get():
            if event.type in [QUIT, K_ESCAPE]:
                pygame.quit()
                sys.exit()
            if event.type == MOUSEBUTTONDOWN:
                clicking = True
            if event.type == KEYDOWN:
                if event.key == K_q:
                    pygame.quit()
                    sys.exit
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
        self.screens[self.curr_screen]['func'](mouse_pos, clicking)
        pygame.display.update()
    
    def start_screen(self, mpos, clicking): # setup params n whatnot
        # start
        for id, data in self.screens['start']['btns'].items():
            data['toggle'] = draw_button(
                self.screen,
                *data['params'],
                data['toggle'],
                mpos, 
                clicking
            )
        
    
    def play_screen(self, pos, clicking): # timer
        # TODO: the white half of the timer i think         
        draw_rect(
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
                pygame.quit()
                sys.exit()
            
            elif cmd_tokens[0] == 'switch':
                self.is_player_turn ^= True # dont ask
        

    def end_screen(self, pos, clicking): # gameover / cleanup n whatnot
        pass


def main(args=None):
    rclpy.init(args=args)

    ui = UIController()
    rclpy.spin(ui)

    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
