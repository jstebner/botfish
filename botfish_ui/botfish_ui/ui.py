import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multiprocessing import Queue

import pygame
from pygame.locals import *

class UIController(Node):
    DIM = (1920//2, 1080//2) # rmv //2 for final
    PERIOD_s = 1/32 # 32 UpS
    
    def __init__(self):
        pygame.init()
        super().__init__('ui_controller')
        # self.screen = pygame.display.set_mode(DIM, pygame.NOFRAME) # final
        self.screen = pygame.display.set_mode(self.DIM,) # testing
        pygame.display.set_caption('Botfish Chess Timer')

        self.i = 0
        self.x = 0
        self.q = Queue()
        
        self.pub = self.create_publisher(
            String,
            'ui_ping',
            10
        )
        self.sub = self.create_subscription(
            String,
            'debug_cmd',
            self.q.put, # puts received msg in q directly
            10
        )
        self.timer = self.create_timer(
            self.PERIOD_s,
            self.update
        )


    def update(self):
        # TODO: make this look good
        self.screen.fill((0,0,0))
        self.x += 5
        self.x %= 1080
        box = pygame.Rect(self.x, 30, 20, 20)
        pygame.draw.rect(self.screen,(64,31,255),box)
        
        while not self.q.empty():
            cmd_tokens = self.q.get().data.split()
            if cmd_tokens[0] == 'stop':
                pygame.quit()
                sys.exit()
        
        for event in pygame.event.get():
            if event.type in [QUIT, K_ESCAPE]:
                # cleanup here
                pygame.quit()
                sys.exit()
            if event.type == MOUSEBUTTONDOWN:
                self.i += 1
                ping = String()
                ping.data = f'ping {self.i}'
                print(ping.data)
                self.pub.publish(ping)
        
        pygame.display.update()
        

def main(args=None):
    print(type(args))
    rclpy.init(args=args)

    ui = UIController()
    rclpy.spin(ui)

    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
