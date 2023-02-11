import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multiprocessing import Process, Pipe

import pygame
from pygame.locals import *

class UIController(Node):
    DIM = (1920//2, 1080//2) # rmv //2 for final
    PERIOD_S = 1/32 # 32 UpS
    
    def __init__(self):
        super().__init__('ui_controller')
        self.pub = self.create_publisher(
            String,
            'ui_ping',
            10
        )
        self.sub = self.create_subscription(
            String,
            'debug_cmd',
            self.receive_cmd,
            10
        )

        self.timer = self.create_timer(
            self.PERIOD_S,
            self.update
        )


        pygame.init()
        # self.screen = pygame.display.set_mode(DIM, pygame.NOFRAME) # final
        self.screen = pygame.display.set_mode(self.DIM,) # testing

        self.i = 0
        self.x = 0

    def update(self):
        self.screen.fill((0,0,0))
        self.x += 0.5
        self.x %= 1080
        box = pygame.Rect(self.x, 30, 20, 20)
        pygame.draw.rect(self.screen,(64,31,255),box)
        pygame.display.update()

        # cmd = cout.recv()
        # if cmd:
        #     print(cmd)
        
        for event in pygame.event.get():
            if event.type == QUIT:
                # cleanup here
                pygame.quit()
                sys.exit()
            if event.type == MOUSEBUTTONDOWN:
                self.i += 1
                ping = String()
                ping.data = f'ping! {self.i}'
                print('ping!')
                self.pub.publish(ping)
        
        pygame.display.flip()

    def timer_callback(self):
        msg = String()
        msg.data = 'ping'
        self.pub.publish(msg)
        print('published ping from node')
    
    def receive_cmd(self, msg):
        print(f'received: {msg.data}')
        

def main(args=None):
    print(type(args))
    rclpy.init(args=args)

    ui = UIController()
    rclpy.spin(ui)

    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
