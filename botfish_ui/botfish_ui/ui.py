import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pygame
from pygame.locals import *

class UIController(Node):
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
        self.main()
    
    def receive_cmd(self, msg):
        print(f'received: {msg.data}')
    
    def main(self):
        pygame.init()
        # screen = pygame.display.set_mode((1920, 1080), pygame.NOFRAME) # final
        screen = pygame.display.set_mode((1920//2, 1080//2),) # testing

        i = 0
        while True:
            screen.fill((0,0,0))
            box = pygame.Rect(10, 10, 20, 20)
            pygame.draw.rect(screen,(64,31,255),box)
            pygame.display.update()

            for event in pygame.event.get():
                if event.type == QUIT:
                    # cleanup here
                    pygame.quit()
                    sys.exit()
                if event.type == MOUSEBUTTONDOWN:
                    i += 1
                    ping = String()
                    ping.data = f'ping! {i}'
                    self.pub.publish(ping)
            
            pygame.display.flip()



def main(args=None):
    rclpy.init(args=args)

    ui = UIController()
    rclpy.spin(ui)

    ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
