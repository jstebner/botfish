import sys
from subprocess import Popen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


PERIOD_s = 1/10 # 10 UpS
class DebugTerminal(Node):
    def __init__(self):
        start_display()
        
        super().__init__('debug_terminal')
        self.debug_cmd_pub = self.create_publisher(
            String,
            'debug_cmd',
            10
        )
        self.player_move_pub = self.create_publisher(
            String,
            'player_move',
            10
        )
        self.timer = self.create_timer(
            PERIOD_s,
            self.update
        )
    
    def _uci_check(self, s):
        # TODO: make this
        return True

    def update(self):
        msg = String()
        # TODO: receive input from somewhere or smthn idk
        # while self.q:
        #     print(self.q.pop(0))

        cmd_tokens = input('botfish-debug:~$ ').lower().split()
        if not cmd_tokens:
            return
        
        # cmds that need parsing
        elif cmd_tokens[0] == 'help':
            # stop, stopall, push (uci), pop (int), help, switch, emote (key), svn
            print('you got this lil bro :)')

        elif cmd_tokens[0] in ['stop', 'stopall']:
            msg.data = cmd_tokens[0]
            self.debug_cmd_pub.publish(msg)
            sys.exit()
        
        elif cmd_tokens[0] == 'display':
            if len(cmd_tokens) != 1:
                print('display dont take args')
                return
            start_display()

        elif cmd_tokens[0] == 'svn': # spoof vision node
            if len(cmd_tokens) == 1:
                print('need move to spoof')
                return
            if len(cmd_tokens) > 2:
                print('too many args')
                return
            if not self._uci_check(cmd_tokens[1]):
                print('move is invalid')
                return
            
            msg.data = f'push {cmd_tokens[1]}'
            self.player_move_pub.publish(msg)
            return

        elif cmd_tokens[0] == 'push':
            if len(cmd_tokens) == 1:
                print('need a move to push bozo')
                return
            ok = True
            for move in cmd_tokens[1:]:
                if not self._uci_check(move):
                    ok = False
                    print(f'{move} is invalid')
            if not ok:
                return
        
        elif cmd_tokens[0] == 'pop':
            if len(cmd_tokens) > 2:
                print('too many jawns bruv')
                return
            elif len(cmd_tokens) == 1:
                cmd_tokens.append('1')
            elif not cmd_tokens[1].isdigit():
                print('gimme a num idot')
                return
        
        elif cmd_tokens[0] == 'emote':
            if len(cmd_tokens) == 1:
                print('need which emote man')
                return
            if len(cmd_tokens) > 2:
                print('too many args')
                return
            if cmd_tokens[1] == '--list':
                print('available emotes:\n -wave\n -calibrate') # TODO: update as necessary
                return
            if cmd_tokens[1] not in [ # TODO: update as necessary
                'wave',
                'calibrate'
            ]:
                print('invalid emote')
                return

        else:
            print('huh')
        
        msg.data = ' '.join(cmd_tokens)
        self.debug_cmd_pub.publish(msg)

def start_display():
    # please oh please for the love of God dont judge me for this
    Popen('ros2 run botfish_debug display', shell=True)

def main(args=None):
    # print(__file__)
    rclpy.init(args=args)

    node = DebugTerminal()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
