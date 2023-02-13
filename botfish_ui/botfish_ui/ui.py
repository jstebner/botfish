import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multiprocessing import Queue

import pygame
from pygame.locals import *

DIM = (1920//2, 1080//2) # rmv //2 for final
PERIOD_s = 1/32 # 32 UpS

class UIController(Node):
    
    def __init__(self):
        pygame.init()
        super().__init__('ui_controller')
        # self.screen = pygame.display.set_mode(DIM, pygame.NOFRAME) # final
        self.screen = pygame.display.set_mode(DIM,) # testing
        pygame.display.set_caption('Botfish Chess Timer')

        self.i = 0
        self.debug_q = Queue()
        self.gamestate_q = Queue()
        self.is_player_turn = True # TODO: make this depend on the game or whatev
        
        self.ui_ping_pub = self.create_publisher(
            String,
            'ui_ping',
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
            self.update
        )


    def update(self):
        # TODO: make this look good
        self.screen.fill((0,0,0))
        
        # event listener # TODO: maybe make this its own func or smthn
        for event in pygame.event.get():
            if event.type in [QUIT, K_ESCAPE]:
                # cleanup here
                pygame.quit()
                sys.exit()
                
            if event.type == MOUSEBUTTONDOWN:
                pass # TODO: idk bro
            
            if event.type == KEYDOWN:
                if event.key == K_q:
                    pygame.quit()
                    sys.exit
                if event.key == K_SPACE:
                    if self.is_player_turn:
                        self.i += 1
                        ping = String()
                        ping.data = f'ping {self.i}'
                        print(ping.data)
                        self.ui_ping_pub.publish(ping)
                        self.is_player_turn = False

        # TODO: the white half of the timer i think         
        pygame.draw.rect(
            self.screen, 
            (255,255,255,255),
            pygame.Rect(0, 0, DIM[0]//2, DIM[1])
        )
        
        # TODO: mfin timers n whatnot
        
        # process debug queue
        while not self.debug_q.empty():
            cmd_tokens = self.debug_q.get().data.split()
            if cmd_tokens[0] == 'stop':
                pygame.quit()
                sys.exit()
        
        
        
        pygame.display.update()
        

def main(args=None):
    rclpy.init(args=args)

    ui = UIController()
    rclpy.spin(ui)

    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
