#
import cv2
from glob import glob
import numpy as np
import matplotlib.pyplot as plt

Mat = np.ndarray[int, np.dtype[np.generic]]

class BoardDetector:
    def __init__(self):
        """Turns consectutive images of a chess game and returns board states for each
        """
        self.board = None
    
    def predict(self, img: Mat) -> str:
        """predicts board state of image given previous board state

        Args:
            img (Mat): cv2.imread object of image TODO: update this to include depth channel

        Returns:
            str: predicted board state in FEN
        """
        blur = cv2.GaussianBlur(img, (5,5), 0)
        _, img_binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        img = cv2.resize(img, (img.shape[0]//5,img.shape[1]//5))
        img_binary = cv2.resize(img_binary, (img_binary.shape[0]//5,img_binary.shape[1]//5))
        cv2.imshow('img',img)
        cv2.imshow('bin',img_binary)
        cv2.waitKey(0)
        
        return None
    
    
if __name__ == '__main__':
    detector = BoardDetector()
    
    images = sorted(glob('./test/*.png'))
    for image in images:
        img = cv2.imread(image, 0)
        print(detector.predict(img))
        break