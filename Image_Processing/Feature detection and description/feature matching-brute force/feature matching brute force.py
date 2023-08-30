import cv2
import numpy as np

img1=cv2.imread("200INR.jpg",0)
cap=cv2.VideoCapture(0)

while(True):
    _,img2=cap.read()
    img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

    #ORB Detector
    orb=cv2.ORB_create()
    kp1,des1=orb.detectAndCompute(img1,None)#mask=none
    kp2,des2=orb.detectAndCompute(img2,None)#mask=none

    #Brute Force Matching
    bf=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)#cross check is by default false if true then only best matching features are detected
    matches=bf.match(des1,des2)

    #no. of matches found
    #print(len(matches))

    #Smaller the distance the better the match
    #for m in matches:
    #    print(m.distance)

    #we want only the best matches so we sort them low to high on distance
    matches=sorted(matches,key=lambda x:x.distance)

    #draw the matches
    matching_result=cv2.drawMatches(img1,kp1,img2,kp2,matches[:20],None,flags=2)
    
    
    cv2.imshow("img1",img1)
    cv2.imshow("img2",img2)
    cv2.imshow("Matching Result",matching_result)
    k=cv2.waitKey(1)
    if k==27:
        break

cap.release()
cv2.destroyAllWindows()

