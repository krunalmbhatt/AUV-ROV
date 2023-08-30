import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread("D:/NISARG/PICS/IMG_20190803_125719.jpg")
b,g,r=cv2.split(img)

cv2.imshow('b',b)
cv2.imshow('g',g)
cv2.imshow('r',r)


#plot histogram using matplotlib
plt.hist(b.ravel(),256,[0,256])
plt.hist(g.ravel(),256,[0,256])
plt.hist(r.ravel(),256,[0,256])

plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()
