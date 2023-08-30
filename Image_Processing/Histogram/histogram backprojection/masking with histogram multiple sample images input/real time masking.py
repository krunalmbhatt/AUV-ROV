#increase erode kernel size to reduce the noice but that also reduces the usefull mask so increase morph ellipse kernel size

import cv2
import numpy as np
from matplotlib import pyplot as plt

#read images
cap=cv2.VideoCapture(0)
roi=cv2.imread("dataset images/1.jpg")
roi1=cv2.imread("dataset images/2.jpg")
roi2=cv2.imread("dataset images/3.jpg")

while(True):

    _,original_image=cap.read()
    #original_image=cv2.GaussianBlur(original_image, (2, 2), 0)
    hsv_original=cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
    
    hsv_roi=cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hsv_roi1=cv2.cvtColor(roi1, cv2.COLOR_BGR2HSV)
    hsv_roi2=cv2.cvtColor(roi2, cv2.COLOR_BGR2HSV)

    #histogram of hue and sat. of roi 
    roi_hist=cv2.calcHist([hsv_roi],[0,1],None,[180,256],[0,180,0,256])
    roi1_hist=cv2.calcHist([hsv_roi1],[0,1],None,[180,256],[0,180,0,256])
    roi2_hist=cv2.calcHist([hsv_roi2],[0,1],None,[180,256],[0,180,0,256])

    #split channels of roi
    hue,saturation,value=cv2.split(hsv_roi)
    hue1,saturation1,value1=cv2.split(hsv_roi1)
    hue2,saturation2,value2=cv2.split(hsv_roi2)


    #print to find range of hue
    #for h in hue:
     #   print(h)
    #for s in saturation:
     #   print(s)
    #for v in value:
     #   print(min(v))


    mask=cv2.calcBackProject([hsv_original],[0,1],roi_hist,[0,180,0,256],1)
    mask1=cv2.calcBackProject([hsv_original],[0,1],roi1_hist,[0,180,0,256],1)
    mask2=cv2.calcBackProject([hsv_original],[0,1],roi2_hist,[0,180,0,256],1)
    
    kernel = np.ones((1,1),np.uint8)#increase to remove noise but detection range will decrease as the mask will get smaller than erode kernel it will be removed
    kernel1 = np.ones((1,1),np.uint8)
    kernel2 = np.ones((1,1),np.uint8)
    
    mask = cv2.erode(mask,kernel,iterations = 1)
    mask1 = cv2.erode(mask1,kernel1,iterations = 1)
    mask2 = cv2.erode(mask2,kernel2,iterations = 1)

    #remove noise
    kernel_ellipse=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))#increase for more mask
    #for k in kernel:
        #print(k)
    mask=cv2.filter2D(mask,-1,kernel_ellipse)
    mask1=cv2.filter2D(mask1,-1,kernel_ellipse)
    mask2=cv2.filter2D(mask2,-1,kernel_ellipse)


    _,mask=cv2.threshold(mask,60,255,cv2.THRESH_BINARY)
    _,mask1=cv2.threshold(mask1,60,255,cv2.THRESH_BINARY)
    _,mask2=cv2.threshold(mask2,60,255,cv2.THRESH_BINARY)


    #merge detection of each mask in one try and/or for tuning or:merges/and:common
    mask=cv2.bitwise_and(mask,mask1,mask2)

    #create contour
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)#(image,contour retrival mode,contour approximation methord)

    #find index of greatest contour
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area>=1000:
           cv2.drawContours(original_image, cnt, -1, (0, 255, 0),2)#(image on which to draw contour,contours,index of contour(-1 for all),color,thickness)

    #make mask of three channel to add with original img
    mask_3channel=cv2.merge((mask,mask,mask))

    result=cv2.bitwise_and(original_image,mask_3channel)
    cv2.imshow("result",result)
    cv2.imshow("mask",mask2)
    cv2.imshow("original image",original_image)
    #cv2.imshow("roi",roi)
    #plt.imshow(roi_hist)
    #plt.show()
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()





