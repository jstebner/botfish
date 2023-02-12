import os
from sys import argv
from getopt import getopt
from stockfish import Stockfish
import pygame
from pygame.locals import *
import time
import random

class Botfish:
    def __init__(self, engine: str, lvl: int, clr: chr, side: chr):
        """botfish main program

        Args:
            engine (str): path of stockfish engine
            lvl (int):  difficulty of the robot
                            0: ez
                            1: mid (default)
                            2: ultra nightmare mode
            clr (chr): color bot is playing:
                            'W': white (default)
                            'B': black
            side (chr): side bot is playing (relative to computer):
                            'L': left (default)
                            'R': right
        """
        self.sf = Stockfish(
            path=engine,
            parameters={
                'UCI_Elo':{0:500, 1:1000, 2:2500}[lvl],
                'UCI_LimitStrength': True,
                'Slow Mover': 0,
                'Ponder': True
                }
            )
        self.clr = clr
        self.delay_s = (2-lvl) * 5
            
    def process_player_move(self, move: str) -> tuple[int, str]:
        """update board state with player move and return stockfish move

        Args:
            move (str): move performed: "{from_coord}{to_coord}"; ex: "a2a3"

        Returns:
            tuple[int, str]: 
                int: response code:
                    0: EXIT
                    1: ERROR
                    2: OKAY
                str: response string (move or error message)
        """

        # NOTE: process input
        # stopwords
        if any([stopword in move for stopword in ('exit', 'quit', 'stop', 'end', 'LET ME OUT OF THIS CHAIR YOU IDIOT')]): 
            return (0, 'stopword received')
        
        # legal move check
        if not self.sf.is_move_correct(move):
            return (1,'you idiot')

        self.sf.make_moves_from_current_position([move])
        print(self.sf.get_board_visual()) # TODO: rmv this
        
        # NOTE: stockfish move
        move = self.sf.get_best_move()
        self.sf.make_moves_from_current_position([move])
        return (2, move)
    
    # TODO: all this jawn
    def send_bot_move(self, move: str) -> None: # REVIEW: return type of this
        """send chess move to manip node

        Args:
            move (str): move performed: "{from_coord}{to_coord}"; ex: "a2a3"
        """
        print("sending move:", move)
        time.sleep(random.normalvariate(mu=self.delay_s, sigma=1)) # TODO: make this not suck ass or smthn
        pass
    
    # TODO: all this jawn as well
    def get_player_move(self) -> str:
        """retrieve move made by player

        Returns:
            str: move performed: "{from_coord}{to_coord}"; ex: "a2a3"
        """
        return input('mkae ya move: ').lower() # TODO: replace this with viz node
        
    def main(self):
        pygame.init()
        # DEBUG: change resolution
        # screen = pygame.display.set_mode((1920, 1080), pygame.NOFRAME) # final
        screen = pygame.display.set_mode((1920//2, 1080//2),) # testing

        # TODO: uhhhhhhhhhhhh
        # user press done btn -> 
        # bot take pic -> 
        # pic sent over network to client pc -> 
        # pic processed by 2fen -> 
        # get move somehow (find out 2fen return type and turn that into standard chess notation for move performed) -> 
        # give move to sf -> 
        # sf return its move -> 
        # move translated to trajectory (make a module for translating move to trajectory, maybe using corner data from 2fen for localization) -> 
        # trajectory sent to bot ->
        # bot make move ->
        # bot wait for user to hit btn.
        print(self.sf.get_parameters())

        if self.clr == 'W':
            move = self.sf.get_best_move()
            self.send_bot_move(move)
            self.sf.make_moves_from_current_position([move])
        
        move = ''
        running = True
        while running:
            # TODO: ui stuffs 
            screen.fill((0,0,0))
            avatar = pygame.Rect(0, 0, 20, 20)
            pygame.draw.rect(screen,(64,31,255),avatar)
            pygame.display.update()
            for event in pygame.event.get():
                pass
            
            print(self.sf.get_board_visual()) # TODO: rmv this

            move = self.get_player_move()
            status, response = self.process_player_move(move)
            if status == 0: # exit
                print("stopword received")
                exit()
            if status == 1: # error
                print(response)
            if status == 2: # okay
                self.send_bot_move(move)
                print(self.sf.get_board_visual()) # TODO: rmv this
            
            
            pygame.display.update()
            
        pygame.quit()

def main(args=None):
    # move cwd to */botfish/
    import pathlib
    parent_dir = pathlib.Path(__file__).parent.resolve()
    os.chdir(parent_dir)
    
    # check for valid os (ik we cant use win but shut up ok)
    import sys
    # print(sys.platform)
    if 'win' in sys.platform:
        engine = './engines/stockfish-windows-2022-x86-64-avx2.exe'
    elif 'linux' in sys.platform:
        engine = './engines/stockfish-ubuntu-20.04-x86-64'
    else:
        print('OS not supported')
        exit()
    
    # default parameters
    level = 1 # 0: ez, 1: med, 2: sicko mode
    color = 'W'
    side = 'L'
    
    # parse arguments
    try:
        shortopts = 'hl:c:s:'
        longopts = ['help','level=', 'color=', 'side=']
        args, vals = getopt(argv[1:], shortopts=shortopts, longopts=longopts)
        print(args, vals)
        
        for curr_arg, curr_val in args:
            if curr_arg in ('-h','--help'):
                print('TODO: help screen')
                exit()

            elif curr_arg in ('-l','--level'):
                level = int(curr_val)
                if level not in (0,1,2):
                    raise Exception('level must be 0, 1, or 2')
                
            elif curr_arg in ('-c','--color'):
                if curr_val.upper() not in ('W', 'B'):
                    raise Exception('color must be W or B')
                color = curr_val.upper()
            
            elif curr_arg in ('-s', '--side'):
                if curr_val.upper() not in ('L', 'R'):
                    raise Exception('side must be L or R')
                side = curr_val.upper()
    except Exception as e:
        print(e)
        exit()
        
    botfish = Botfish(engine, level, color, side)
    botfish.main()

if __name__ == '__main__':
    main()