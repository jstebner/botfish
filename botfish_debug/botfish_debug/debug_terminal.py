import sys
from subprocess import Popen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DebugTerminal(Node):
    PERIOD_s = 1/32 # 32 UpS
    
    def __init__(self):
        super().__init__('debug_terminal')
        self.debug_cmd_pub = self.create_publisher(
            String,
            'debug_cmd',
            10
        )
        self.update_debug_board_pub = self.create_publisher(
            String,
            'update_debug_board',
            10
        )
        self.timer = self.create_timer(
            self.PERIOD_s,
            self.update
        )

    def update(self):
        msg = String()
        # TODO: somthin to do with a queue
        # while self.q:
        #     print(self.q.pop(0))

        cmd_tokens = input('botfish-debug:~$ ').lower().split()
        if not cmd_tokens:
            return
        
        # cmds that need parsing
        # TODO: add calibrate command
        elif cmd_tokens[0] == 'help':
            # TODO: this, also list topics or smthn
            # stop, stopall, push (uci), pop (int), help, switch
            print('you got this bro :)')

        elif cmd_tokens[0] in ['stop', 'stopall']:
            msg.data = 'stop'
            self.update_debug_board_pub.publish(msg)
            if cmd_tokens[0] == 'stopall':
                self.debug_cmd_pub.publish(msg)
            sys.exit()

        elif cmd_tokens[0] == 'push':
            if len(cmd_tokens) == 1:
                print('need a move to push bozo')
                return
            # TODO: check each arg for ok
            for move in cmd_tokens[1:]:
                pass
            msg.data = ' '.join(cmd_tokens)
            self.update_debug_board_pub.publish(msg)
        
        elif cmd_tokens[0] == 'pop':
            if len(cmd_tokens) > 2:
                print('too many jawns bruv')
                return
            elif len(cmd_tokens) == 1:
                cmd_tokens.append('1')
            elif not cmd_tokens[1].isdigit():
                print('gimme a num idot')
                return
            
            msg.data = ' '.join(cmd_tokens)
            self.update_debug_board_pub.publish(msg)

        # anything else
        else:
            msg.data = ' '.join(cmd_tokens)
            self.debug_cmd_pub.publish(msg)


def main(args=None):
    # please oh please for the love of God dont judge me for this
    Popen('ros2 run botfish_debug display', shell=True)
    
    rclpy.init(args=args)

    node = DebugTerminal()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
