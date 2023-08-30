import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('D:/NISARG/WALL PAPER/messi5.jpg',0)
ret,thresh1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
ret,thresh2 = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
ret,thresh3 = cv2.threshold(img,127,255,cv2.THRESH_TRUNC)
ret,thresh4 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO)
ret,thresh5 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO_INV)

titles = ['Original Image','BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']
images = [img, thresh1, thresh2, thresh3, thresh4, thresh5]

cv2.imshow('Original Image',img)

cv2.imshow('BINARY',thresh1)

cv2.imshow('BINARY_INV',thresh2)

cv2.imshow('TRUNC',thresh3)

cv2.imshow('TOZERO',thresh4)

cv2.imshow('TOZERO_INV',thresh5)
