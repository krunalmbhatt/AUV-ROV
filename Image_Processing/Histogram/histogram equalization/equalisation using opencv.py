import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('img.jpg',0)
equ = cv2.equalizeHist(img)

cv2.imshow('img',img)
cv2.imshow('equ',equ)
