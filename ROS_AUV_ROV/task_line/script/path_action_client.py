import rospy
from __future__ import print_function

import actionlib

import actionlib_path.msg #for message being used like goal msg and result msg


import cv2
import numpy as np
import math

def path_client():
    client=actionlib.SimpleActionClient('path',actionlib_path.msg.pathAction)
    client.wait_for_server()
    goal=actionlib_path.msg.pathGoal(order=20)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()
       
if __name__=='__main__':
    rospy.init_node('path_client_py')
    result=path_client()

    cap=cv2.VideoCapture(1)

    while (cap.isOpened()):
        _, frame=cap.read()
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        ret,thresh1=cv2.threshold(hsv,110,255,cv2.THRESH_BINARY)
        low_a=np.array([0,0,0])
        high_a=np.array([0,0,255])
        mask=cv2.inRange(thresh1,low_a,high_a)
        mask_not=cv2.bitwise_not(mask)
        res=cv2.bitwise_and(thresh1,thresh1,mask=mask)
        imgray=cv2.cvtColor(res,cv2.COLOR_HSV2BGR)
        gray=cv2.cvtColor(imgray,cv2.COLOR_BGR2GRAY)
        image,contours,heirarchy=cv2.findContours(gray,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        res=cv2.drawContours(gray,contours,-1,(255,0,0),-1)
            #low_b=np.array([90,20,120])
            #high_b=np.array([110,60,180])
            #mask1=cv2.inRange(res,low_b,high_b)
            #res2=cv2.bitwise_and(res,res,mask=mask1)
    
        max_area = 0
        best_cnt = 1
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt

        M=cv2.moments(best_cnt)
        cx,cy=int(M['m10']/M['m00']),int(M['m01']/M['m00'])
#       print(cx,cy)
        x=(320,240)
        cv2.circle(frame,(cx,cy),10,(100,255,255),-1)
    
#    x,y,w,h=cv2.boundingRect(best_cnt)
        rect=cv2.minAreaRect(best_cnt)
#        print (rect)
        box=cv2.boxPoints(rect)
        box=np.int0(box)
    
 #   im=cv2.rectangle(frame,(x,y),(x+w,y+h),(0,2550,0),2)
        im=cv2.drawContours(frame,[box],0,(0,0,255),2)
        maxlen=0

#    print (gray[320,240])
        bx,by=box[0]  #bottom right corner
        ax,ay=box[3]  #upper right corner
        cx,cy=box[2] #upper left corner
    #height
        height=math.sqrt((bx-ax)*(bx-ax)+(by-ay)*(by-ay))
    #breadth
        breadth=math.sqrt((cy-ay)*(cy-ay)+(cx-ax)*(cx-ax))
        angle=math.degrees(np.arctan(-(by-ay)/(bx-ax)))
    
        if height<breadth:
            angle=angle+90
        
        print (angle)
#        print(hsv[320,240])
        cv2.line(frame,(320,0),(320,480),(150,100,255),5)
#    cv2.line(frame,(cx,cy),(cx,480),(0,255,0),5)

        cv2.circle(frame,(320,240),10,(255,255,0),-1)
        cv2.circle(hsv,(320,240),10,(255,255,0),-1)
        cv2.imshow("return",hsv)
        cv2.imshow("Hope",frame)
       # pause(0.001)

        if cv2.waitKey(1) & 0xFF==ord('a'):
            break
    
    except rospy.ROSInterruptException:
        print("programm Interrupted")
