import cv2
import numpy
from matplotlib import pyplot

img=cv2.imread('D:/NISARG/PICS/bird.jpg')
bird=img[1361:2189, 2276:3067]
img[1269:2097, 1184:1975] = bird
pyplot.imshow(img, cmap = 'gray', interpolation = 'bicubic')
pyplot.xticks([]), pyplot.yticks([])
pyplot.show()

if k==ord('s'):
    cv2.imwrite('D:/NISARG/PICS/roi.jpg',img)
    cv2.destroyAllWindows()
    


               
