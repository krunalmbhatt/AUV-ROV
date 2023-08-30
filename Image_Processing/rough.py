import cv2
import numpy

img=cv2.imread("D:/NISARG/PICS/bird.jpg")
g=img.copy() #guassian pyramid element
l=img.copy() #laplacian pyramid element

gp_img=[g] #contains bog to small sizes gausian
lp_img=[l] #contains bog to small sizes laplacian

for i in range(6):
    g=cv2.pyrDown(g) 
    gp_img.append(g)

for i in gp_img:
    cv2.imshow("aaa",i)
    
cv2.waitKey(0)
cv2.destroyAllWindows()
