import cv2
import numpy
import matplotlib

img1_weight=0.99
img2_weight=0.01

img1=cv2.imread('D:/NISARG/WALL PAPER/lamborghini-huracan-evo-2019-rear-4k_1547937058.jpg')
img2=cv2.imread('D:/NISARG/WALL PAPER/auto_front_view_motion_blur_119752_3840x2400.jpg')

print('original dimention img1=',img1.shape)
print('original dimention img2=',img2.shape)

width=int(1280)
height=int(720)
dim=(width,height)

img11=cv2.resize(img1,dim,interpolation=cv2.INTER_AREA)
img22=cv2.resize(img2,dim,interpolation=cv2.INTER_AREA)

print("        ")
print("new dimentions img1=",img11.shape)
print("new dimentions img2=",img22.shape)
print("        ")


while(img1_weight>=0 and img2_weight<=1):
    imgsum=cv2.addWeighted(img11,img1_weight,img22,img2_weight,0)
    img2_weight=img2_weight+0.1
    img1_weight=img1_weight-0.1
    cv2.imshow('slideshow',imgsum)
    print(img1_weight)
    print(img2_weight)
    cv2.waitKey(100)

    if img2_weight>=1 and img1_weight<=0:
        while img2_weight>=0 and img1_weight<=1:
            img2_weight=img2_weight-0.1
            img1_weight=img1_weight+0.1
            imgsum=cv2.addWeighted(img11,img1_weight,img22,img2_weight,0)
            print(img1_weight)
            print(img2_weight)
            cv2.waitKey(100)
            cv2.imshow('slideshow',imgsum)

cv2.waitKey(0)
cv2.destroyAllWindows
