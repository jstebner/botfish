from sys import argv
from getopt import getopt
from stockfish import Stockfish
import pygame
from pygame.locals import *

class Botfish:
    def __init__(self, lvl, clr):
        self.sf = Stockfish(
            path='stockfish/stockfish-windows-2022-x86-64-avx2.exe',
            parameters={
                'UCI_Elo':{0:500, 1:1000, 2:2500}[lvl],
                'UCI_LimitStrength': True,
                'Slow Mover': 0,
                'Ponder': True
                }
            )
        self.clr = clr
            
    def process_move(self, move: str) -> tuple[int, str]:
        """update board state with opponent move and return stockfish move

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
        if move in ('exit', 'quit', 'stop', 'end'): 
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
    def send_move(self, move: str) -> None: # REVIEW: return type of this
        """send chess move to manip node

        Args:
            move (str): move performed: "{from_coord}{to_coord}"; ex: "a2a3"
        """
        print("sending move:", move)
        pass
    
    # TODO: all this jawn as well
    def get_move(self) -> str:
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
            self.send_move(move)
            self.sf.make_moves_from_current_position([move])
        
        move = ''
        running = True
        while running:
            # ui stuffs
            screen.fill((0,0,0))
            avatar = pygame.Rect(0, 0, 20, 20)
            pygame.draw.rect(screen,(64,31,255),avatar)
            pygame.display.update()
            for event in pygame.event.get():
                pass
            
            print(self.sf.get_board_visual()) # TODO: rmv this
            # NOTE: player move

            move = self.get_move()
            status, response = self.process_move(move)
            if status == 0: # exit
                print("stopword received")
                exit()
            if status == 1: # error
                print(response)
            if status == 2:
                self.send_move(move)
                print(self.sf.get_board_visual()) # TODO: rmv this
            
            
            pygame.display.update()
            
        pygame.quit()

if __name__ == '__main__':
    # default parameters
    level = 1 # 0: ez, 1: med, 2: sicko mode
    color = 'W'
    
    # parse arguments
    try:
        shortopts = 'hl:c:'
        longopts = ['help','level=', 'color=']
        args, vals = getopt(argv[1:], shortopts=shortopts, longopts=longopts)
        
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
    except Exception as e:
        print(e)
        
    botfish = Botfish(level, color)
    botfish.main()