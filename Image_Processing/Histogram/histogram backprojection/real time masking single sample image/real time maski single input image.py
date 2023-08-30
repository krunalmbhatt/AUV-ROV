#increase erode kernel size to reduce the noice but that also reduces the usefull mask so increase morph ellipse kernel size

import cv2
import numpy as np
from matplotlib import pyplot as plt

#read images
cap=cv2.VideoCapture('test video.mp4')
roi=cv2.imread("yellow.jpg")

while(True):

    _,original_image=cap.read()
    #original_image=cv2.GaussianBlur(original_image, (2, 2), 0)
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
    #for v in value:
     #   print(min(v))


    mask=cv2.calcBackProject([hsv_original],[0,1],roi_hist,[0,180,0,256],1)
    kernel = np.ones((4,4),np.uint8)#increase to remove noise but detection range will decrease as the mask will get smaller than erode kernel it will be removed
    mask = cv2.erode(mask,kernel,iterations = 1)

    #remove noise
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))#increase for more mask
    #for k in kernel:
        #print(k)
    mask=cv2.filter2D(mask,-1,kernel)
    _,mask=cv2.threshold(mask,30,255,cv2.THRESH_BINARY)

    #make mask of three channel to add with original img
    mask_3channel=cv2.merge((mask,mask,mask))

    result=cv2.bitwise_and(original_image,mask_3channel)
    cv2.imshow("result",result)
    cv2.imshow("mask",mask)
    cv2.imshow("original image",original_image)
    cv2.imshow("roi",roi)
    #plt.imshow(roi_hist)
    #plt.show()
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()





