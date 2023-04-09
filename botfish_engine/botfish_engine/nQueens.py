import numpy as np
import itertools as it
import rclpy

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String, UInt8


class Board:
    def __init__(self, n: int = 8):
        
        if n not in range(4,9):
            n = 8
        
        #The nxn size of the board
        self._n = n
        
        #The tiles containing queens where 0 is empty and 1 is a queen
        self._occupancy_matrix = np.zeros(shape = (n,n), dtype = int)
        
        #The safe cells determined by the occupancy matrix. 0 is safe and 1 is not
        self._collision_matrix = np.zeros(shape = (n,n), dtype = int)
    
    #For UI purposes it's convenient to make the board readable to a human      
    def display(self):
    

        print_matrix = ""
        for row in range(self._n):
            for col in range(self._n):
                if self._occupancy_matrix[row][col] == 1:
                    print_matrix += "Q "
                else:
                    print_matrix += "# "
            print_matrix += "\n"
                        
        print(print_matrix)

#Recursively try to place a queen in each row   
def findNQueens(board: Board, row: int = 0):
    
    if row == board._n:
        return True
    
    for col in range(board._n):
        
        #If we can place a queen do so
        if not canPlace(board, row, col):
            continue
        
        #Update the boardstate
        updateBoard(board, row, col)
        
        #Recursively try to place a queen in the next row
        if findNQueens(board, row+1) == True:
            return True
       
        #If we couldn't place a queen, update the boardstate
        updateBoard(board, row, col, False)
        
    return False
        
#Returns true if the a queen can be placed at a cell 
def canPlace(board: Board, row: int, col: int):
    return board._collision_matrix[row][col] == 0

def updateBoard(board: Board, row: int, col: int, forward: bool=True):

    # update the occupancy_matrix
    board._occupancy_matrix[row][col] = forward

    # update the attacking_matrix
    for coordinate in it.product(range(board._n), repeat=2):
        x, y = coordinate
        if(
            # same row/col
            x == row or y == col or  
            # same diagonals
            (x-y) == (row-col) or    
            (x+y) == (row+col)       
           ):
            board._collision_matrix[x][y] += 1 if forward else -1

def nQueens():
    #Initialize a board of size nxn
    n = input('Enter a value 4-8: ')
    board = Board( int(n) )

    #Find a solution space
    findNQueens(board)

    #Print the board
    board.display()

    rclpy.init(args=None)
    node = rclpy.create_node('Matrix_Publisher')
    publisher = node.create_publisher(String, 'tiles',0)

    msg = String()
                
    msg.data = ""

    board2 = Board( int(n) )
    board2._occupancy_matrix = board._occupancy_matrix.transpose(1,0)
            
    for row in range(int(n)):
        for col in range(int(n)):
            if board2._occupancy_matrix[row][col] == 1:
                print( str( chr(row + 65) ) + "," + str( int(n) - col + (8 - int(n) ) ) )
                msg.data += str( ( chr(row + 65)) + str( int(n) - col + (8 - int(n) ) ) )

    node.get_logger().info('Message: "%s"' % msg.data)
    publisher.publish(msg)            
                
    node.destroy_node()
    rclpy.shutdown()

class nQueensNode(Node):
    def __init__(self):
        super().__init__('nqueens_node')
        
        self.tiles_pub = self.create_publisher(
            String,
            'tiles',
            10
        )
        self.perform_nqueens_sub = self.create_subscription(
            UInt8,
            'perform_nqueens',
            self.perform,
            10
        )
    
    def perform(self, n: int):
        if n not in range(4, 9):
            n = 8
        board = Board()
        findNQueens(board)
        board.display()
        msg = String()
        msg.data = ""
        board2 = Board(n)
        board2._occupancy_matrix = board._occupancy_matrix.transpose(1,0)
        
        for row in range(int(n)):
            for col in range(int(n)):
                if board2._occupancy_matrix[row][col] == 1:
                    print( str( chr(row + 65) ) + "," + str( int(n) - col + (8 - int(n) ) ) )
                    msg.data += str( ( chr(row + 65)) + str( int(n) - col + (8 - int(n) ) ) )
        self.tiles_pub.publish(msg)
    

def main(args=None):
    rclpy.init()
    
    node = nQueensNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # NOTE: run nQueens() for one-shot cmd line instance
    # nQueens()
    # NOTE: run main() for continuous node
    main()
    
    
    '''
    #Initialize a board of size nxn
    n = input('Enter a value 4-8: ')
    board = Board( int(n) )

    #Find a solution space
    findNQueens(board)

    #Print the board
    board.display()

    rclpy.init(args=None)
    node = rclpy.create_node('Matrix_Publisher')
    publisher = node.create_publisher(String, 'tiles',0)

    msg = String()
                
    msg.data = ""

    board2 = Board( int(n) )
    board2._occupancy_matrix = board._occupancy_matrix.transpose(1,0)
            
    for row in range(int(n)):
        for col in range(int(n)):
            if board2._occupancy_matrix[row][col] == 1:
                print( str( chr(row + 65) ) + "," + str( int(n) - col + (8 - int(n) ) ) )
                msg.data += str( ( chr(row + 65)) + str( int(n) - col + (8 - int(n) ) ) )

    node.get_logger().info('Message: "%s"' % msg.data)
    publisher.publish(msg)            
                
    node.destroy_node()
    rclpy.shutdown()
    '''
