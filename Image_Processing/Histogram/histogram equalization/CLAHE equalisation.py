import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('sample.jpg',0)

# create a CLAHE object (Arguments are optional).
clahe = cv2.createCLAHE(clipLimit=6.0, tileGridSize=(8,8))
equ = clahe.apply(img)

cv2.imshow('img',img)
cv2.imshow('equ',equ)
