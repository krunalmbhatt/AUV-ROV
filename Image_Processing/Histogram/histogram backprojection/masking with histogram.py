import cv2
import numpy as np
from matplotlib import pyplot as plt

#read images
original_image=cv2.imread("buoy.jpg")
roi=cv2.imread("yellow.jpg")

hsv_original=cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
hsv_roi=cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

#histogram of hue and sat. of roi 
roi_hist=cv2.calcHist([hsv_roi],[0,1],None,[180,256],[0,180,0,256])

#split channels of roi
hue,saturation,value=cv2.split(hsv_roi)
#print to find range of hue
#for h in hue:
 #   print(h)
#for s in saturation:
 #   print(s)
for v in value:
    print(min(v))


mask=cv2.calcBackProject([hsv_original],[0,1],roi_hist,[0,180,0,256],1)

#remove noise
kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
for k in kernel:
    print(k)
mask=cv2.filter2D(mask,-1,kernel)
_,mask=cv2.threshold(mask,30,255,cv2.THRESH_BINARY)

#make mask of three channel to add with original img
mask_3channel=cv2.merge((mask,mask,mask))

result=cv2.bitwise_and(original_image,mask_3channel)
cv2.imshow("result",result)
cv2.imshow("mask",mask)
cv2.imshow("original image",original_image)
cv2.imshow("roi",roi)
plt.imshow(roi_hist)
plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()





