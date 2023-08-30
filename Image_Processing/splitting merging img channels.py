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
print('Resized Dimensions : ',resized.shape)

b,g,r=cv2.split(resized)

print(b)
print(g)
print(r)

resized=cv2.merge((b,g,r))

cv2.imshow('image',resized)
cv2.waitKey(0)
cv2.destroyAllWindows

