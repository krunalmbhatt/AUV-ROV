import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread("D:/NISARG/Image Processing/AUV-Work/Underwater image enhance/samples/sample1.png")
img=cv2.cvtColor(img,cv2.COLOR_BGR2XYZ)
b,g,r=cv2.split(img)


cv2.imshow('b',b)
cv2.imshow('g',g)
cv2.imshow('r',r)


#plot histogram using opencv
hist1=cv2.calcHist([b],[0],None,[256],[0,256])#(image,no of channels 0=grey,mask=none,bins,range)
hist2=cv2.calcHist([g],[0],None,[256],[0,256])#(image,no of channels 0=grey,mask=none,bins,range)
hist3=cv2.calcHist([r],[0],None,[256],[0,256])#(image,no of channels 0=grey,mask=none,bins,range)


plt.plot(hist1)
plt.plot(hist2)
plt.plot(hist3)
plt.show()

k=cv2.waitKey(0)
if k==27:
    cv2.destroyAllWindows()
    quit()
