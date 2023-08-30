import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('sudoku.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


#detect corners
corners = cv2.goodFeaturesToTrack(gray,25,0.01,10)#arguments=(image in gray,no of corners to find,quality between 0 and 1,min dist between two corners)
corners = np.int0(corners)

for i in corners:
    #get coordinates of all corners
    x,y = i.ravel()
    #draw corners
    cv2.circle(img,(x,y),3,255,-1)

plt.imshow(img),plt.show()
