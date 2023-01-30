from sys import argv
from getopt import getopt
from stockfish import Stockfish
import chess

def display(board):
    for i, row in enumerate(str(board).split('\n')):
        print(8-i,'|', row)
    print('  +----------------')
    print('    a b c d e f g h', end='\n\n')
    

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
        self.board = chess.Board()
        self.clr = clr

    def main(self):
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
            move = chess.Move.from_uci(self.sf.get_best_move())
            self.board.push(move)
            self.sf.make_moves_from_current_position([move])
        
        inp = ''
        while True:
            # NOTE: player move 
            display(self.board)
            inp = input('mkae ya move: ').lower()
            if inp in ('exit', 'quit', 'stop', 'end'):
                break
            try:
                move = chess.Move.from_uci(inp)
                # legal move check
                if not self.sf.is_move_correct(move):
                    raise Exception
            except:
                print('you idiot')
                continue

            self.board.push(move)
            self.sf.make_moves_from_current_position([move])
            display(self.board)
            
            # NOTE: sf move
            move = chess.Move.from_uci(self.sf.get_best_move())
            self.board.push(move)
            self.sf.make_moves_from_current_position([move])
            print('feesh move:', move)

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