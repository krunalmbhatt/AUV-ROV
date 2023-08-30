import cv2
import numpy as np
from matplotlib import pyplot as plt

cap=cv2.VideoCapture(0)
def nothing(a):
    pass

cv2.namedWindow('TrackBar')
cv2.createTrackbar('ClipSize','TrackBar',1,30,nothing)
cv2.createTrackbar('GridSize','TrackBar',1,30,nothing)

while(True):

    _,img=cap.read()
    b,g,r=cv2.split(img)

    #normal
    #equ_b = cv2.equalizeHist(b)
    #equ_g = cv2.equalizeHist(g)
    #equ_r = cv2.equalizeHist(r)
    clip=cv2.getTrackbarPos('ClipSize','TrackBar')
    x=cv2.getTrackbarPos('x','TrackBar')
    #clahe
    titleGridSize=(x,x)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(10,10))
    equ_b = clahe.apply(b)
    equ_g = clahe.apply(g)
    equ_r = clahe.apply(r)

    equ=cv2.merge((equ_b,equ_g,equ_r))

    cv2.imshow('img',img)
    cv2.imshow('equ',equ)

    cv2.waitKey(1)

cv2.destroyAllWindows()
