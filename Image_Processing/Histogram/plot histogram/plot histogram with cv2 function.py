import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread("D:/NISARG/Image Processing/AUV-Work/Underwater image enhance/samples/sample1.png",0)

hist=cv2.calcHist([img],[0],None,[256],[0,256])#(image,no of channels 0=grey,mask=none,bins,range)
plt.plot(hist)

plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()
