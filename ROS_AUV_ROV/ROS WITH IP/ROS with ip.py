from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import UInt16
import cv2
import numpy as np
import time 
from simple_pid import PID
import rospy
def callback(x):
    pass

pub_lr = rospy.Publisher('pwm_lr_topic', Int32, queue_size=10)
pub_ud = rospy.Publisher('pwm_ud_topic', Int32, queue_size=10)
rospy.init_node('img_pwm', anonymous=True)
rate = rospy.Rate(10)


TargetX = 210
P = 0.7
I = 0
D = 0.

x_pid = PID()
x_pid.Kp = P
x_pid.Kd = D
x_pid.Ki = I
x_pid.setpoint = TargetX
x_pid.sample_time = 0.01
x_pid.output_limits = (-400, 400)


y_pid = PID()
y_pid.Kp = P
y_pid.Kd = D
y_pid.Ki = I
y_pid.setpoint = TargetX
y_pid.sample_time = 0.01
y_pid.output_limits = (-400, 400)

cap = cv2.VideoCapture(0)
cv2.namedWindow('image')

ilowH = 0
ihighH = 255

ilowS = 0
ihighS = 255

ilowV = 0
ihighV = 255

cv2.createTrackbar('lowH','image',ilowH,255,callback)
cv2.createTrackbar('highH','image',ihighH,255,callback)

cv2.createTrackbar('lowS','image',ilowS,255,callback)
cv2.createTrackbar('highS','image',ihighS,255,callback)

cv2.createTrackbar('lowV','image',ilowV,255,callback)
cv2.createTrackbar('highV','image',ihighV,255,callback)

while not rospy.is_shutdown():
    pwm_lr = 1500
    pwm_ud = 1500
    pid_y = 0
    pid_x = 0
    _, frame = cap.read()
    frame2 = frame
    ilowH = cv2.getTrackbarPos('lowH', 'image')
    ihighH = cv2.getTrackbarPos('highH', 'image')
    ilowS = cv2.getTrackbarPos('lowS', 'image')
    ihighS = cv2.getTrackbarPos('highS', 'image')
    ilowV = cv2.getTrackbarPos('lowV', 'image')
    ihighV = cv2.getTrackbarPos('highV', 'image')

    frame = cv2.resize(frame, (420, 420))
    frame2 = cv2.resize(frame2, (420, 420))

    b, g, r = cv2.split(frame)
    b = frame[:, :, 0]//2
    g = frame[:, :, 1]//2
    r = frame[:, :, 2]

    frame = cv2.merge((b, g, r))

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame2 = cv2.line(frame2, (210, 0), (210, 420), (0,0,0), 3)
    frame2 = cv2.line(frame2, (0, 210), (420, 210), (0,0,0), 3)

    lower_hsv = np.array([ilowH, ilowS, ilowV])
    higher_hsv = np.array([ihighH, ihighS, ihighV])

    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

    frame = np.array(255 * (frame / 255) ** 0.6, dtype='uint8')
    kernel = np.ones((5,5), np.uint8)
    dilated = cv2.dilate(mask,kernel)

    contours, hierarchy = cv2.findContours(dilated , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + (w/2)
            cy = y + (h/2)

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            img = cv2.circle(frame2, (cx, cy), 3, (255, 0, 0), -1)
            img = cv2.drawContours(frame2, [box], 0, (0, 255, 0), 2)
            cv2.putText(img, "RED color", (50 ,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0))
            x_output = x_pid(cx)
            y_output = y_pid(cy)
            pid_x = int(x_output)
            pid_y = int(y_output)
            pwm_lr = 1500 + pid_x
            pwm_ud = 1500 + pid_y

    frame = cv2.bitwise_and(hsv, hsv, mask=mask)

    #ret,thrshed = cv2.threshold(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    #contours,hier = cv2.findContours(thrshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    cv2.imshow('frame',frame2)
    #print(cx, cy)
    pub_lr.publish(pwm_lr)
    pub_ud.publish(pwm_ud)
    rate.sleep()

    print("lpwm: ", pwm_x_1)
    print("rpwm: ", pwm_x_2)
    print("upwm: ". pwm_y)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
"""
for yellow using HLS
lowH=0
HighH=101
lowS=135
highS=166
lowV=48
highV=255
"""
