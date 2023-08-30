import numpy
import cv2
from matplotlib import pyplot

img=cv2.imread('D:/NISARG/WALL PAPER/lamborghini-huracan-evo-2019-rear-4k_1547937058.jpg')

print('original dimentions=',img.shape)

scale_percent=30
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)

resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
print('Resized Dimensions=',resized.shape)

b=img[:,:,0]
g=img[:,:,1]
r=img[:,:,2]

print(b)
print(g)
print(r)
