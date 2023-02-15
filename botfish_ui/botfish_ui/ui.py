import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import Queue

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # why couldnt this just be an arg bro
import pygame
from pygame.locals import *

# UTILS
pygame.init() # initing here cuz i need for global font
SD = 2 # scaledown # TODO: change to 1 for final
DIM = (1920//SD, 1080//SD)
EXPAND = 1.16 # make btn go big by 10%
EXPAND = 1 + (EXPAND-1)/SD
SMOLFONT = pygame.font.SysFont('monospace', 30//SD)
BEEGFONT = pygame.font.SysFont('monospace', 60//SD)
CLRS = { # TODO: make these better and more
    'white': (255,255,255),
    'black': (0,0,0),
    'gray50': (50,50,50),
    'gray150': (150,150,150),
    'gray200': (200,200,200)
}
def scaler(*args):
    return [arg//SD for arg in args]
def unscaler(*args): # you have no right to justdge me for this
    return [arg*SD for arg in args]

# make this better or smthn idk
def close():
    pygame.quit()
    sys.exit()

PERIOD_s = 1/32 # 32 UpS
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
        self.screens = {
            'start': {
                'func': self.start_screen,
                'btns': {
                    # TEST button
                    'test1': {
                        'toggle' : False, # default value
                        'params' : (
                            "test", # display text
                            (50, 50), # position
                            (200,100) # dimension
                        ),
                        # TODO: make the buttons turn other ones on and off and whatever
                        'enables_when_off' : [], # turn this boy on when he go off
                        'disables_when_on' : ['test2', 'test3'], # turn these guys off when he go on
                    },
                    'test2': {
                        'toggle' : False,
                        'params' : (
                            "teeest",
                            (50, 150),
                            (200,100)
                        ),
                        'enables_when_off' : [],
                        'disables_when_on' : ['test'],
                    },
                    'test3': {
                        'toggle' : False,
                        'params' : (
                            "teeeeest", # display text
                            (50, 250), # position
                            (200,100) # dimension
                        ),
                        'enables_when_off' : [],
                        'disables_when_on' : [],
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
    def _draw_button(self, text: str, pos: tuple, dim: tuple, toggle: bool, mpos: tuple, click: bool) -> bool:
        pos = scaler(*pos) # removes reference 
        dim = scaler(*dim)
        text_pos = (
            int(pos[0] + dim[0]/2 - SMOLFONT.size(text)[0]/2),
            int(pos[1] + dim[1]/2 - SMOLFONT.size(text)[1]/2)
        )

        if toggle:
            b_color = CLRS['white']
            t_color = CLRS['gray50']
        else:
            b_color = CLRS['gray50']
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
                toggle ^= True # dont ask

            return toggle
        
        pygame.draw.rect(self.screen, b_color, btn)
        self._draw_text(text, t_color, *unscaler(*text_pos), SMOLFONT)
        
        return toggle

    def _draw_buttons(self, screen_id, mpos, clicking):
        self.foreground = None
        for id, data in self.screens[screen_id]['btns'].items():
            data['toggle'] = self._draw_button(
                *data['params'],
                data['toggle'],
                mpos, 
                clicking
            )

            
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
        self.screens[self.curr_screen]['func'](mouse_pos, clicking)
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
