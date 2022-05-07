import cv2
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import UInt16
import time
from simple_pid import PID
import rospy
import time

pub_lr = rospy.Publisher('pwm_lr_topic', Int32, queue_size=10)
pub_ud = rospy.Publisher('pwm_ud_topic', Int32, queue_size=10)
rospy.init_node('img_pwm', anonymous=True)
rate = rospy.Rate(10)

def callback():
    pass

cv2.createTrackbar('lowH','image',0,255,callback)
cv2.createTrackbar('highH','image',0,255,callback)

cv2.createTrackbar('lowS','image',0,255,callback)
cv2.createTrackbar('highS','image',0,255,callback)

cv2.createTrackbar('lowV','image',0,255,callback)
cv2.createTrackbar('highV','image',0,255,callback)

cv2.createTrackbar('lowH_gate','image',0,255,callback)
cv2.createTrackbar('highH_gate','image',0,255,callback)

cv2.createTrackbar('lowS_gate','image',0,255,callback)
cv2.createTrackbar('highS_gate','image',0,255,callback)

cv2.createTrackbar('lowV_gate','image',0,255,callback)
cv2.createTrackbar('highV_gate','image',0,255,callback)

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

# Load Yolo
#net = cv2.dnn.readNet("yolov3_training_last.weights", "yolov3_testing.cfg")
# Name custom object
# Images path
#images_path = glob.glob(r"D:\Pysource\Youtube\2020\105) Train Yolo google cloud\dataset\*.jpg")
cap=cv2.VideoCapture(0)
b=0

_,img=cap.read()
center_frame=[int(img.shape[0]/2),int(img.shape[1]/2)]

while not rospy.is_shotdown():
    ilowH = cv2.getTrackbarPos('lowH', 'image')
    ihighH = cv2.getTrackbarPos('highH', 'image')
    ilowS = cv2.getTrackbarPos('lowS', 'image')
    ihighS = cv2.getTrackbarPos('highS', 'image')
    ilowV = cv2.getTrackbarPos('lowV', 'image')
    ihighV = cv2.getTrackbarPos('highV', 'image')

    ilowH_gate = cv2.getTrackbarPos('lowH_gate', 'image')
    ihighH_gate = cv2.getTrackbarPos('highH_gate', 'image')
    ilowS_gate = cv2.getTrackbarPos('lowS_gate', 'image')
    ihighS_gate = cv2.getTrackbarPos('highS_gate', 'image')
    ilowV_gate = cv2.getTrackbarPos('lowV_gate', 'image')
    ihighV_gate = cv2.getTrackbarPos('highV_gate', 'image')
    #frame = cv2.re
    
    pwm_lr = 1500
    pwm_ud = 1500
    pid_y = 0
    pid_x = 0

    b=b+1
    _,img = cap.read()

    frame=img
    frame2=img

    
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
        if (area > 3000):
            x, y, w, h = cv2.boundingRect(contour)
            ccx = x + (w/2)
            ccy = y + (h/2)

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            img = cv2.circle(frame2, (ccx, ccy), 3, (255, 0, 0), -1)
            img = cv2.drawContours(frame2, [box], 0, (0, 255, 0), 2)

            if ccx>=110 and ccx<=310:
                cv2.putText(img, "Obstacle in path", (50 ,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0))
                print("Obstacle Detected")

                while (ccx <110 and ccx>310):
                    if ccx<110:
                        pwm_lr=200
                    if ccy>310:
                        pwn_lr=-200
                    
                ###################
                #go left then go straight then go right
                ############################

    
    frame = cv2.bitwise_and(hsv, hsv, mask=mask)

    #img = cv2.resize(img, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape
    img = cv2.line(img,(int(img.shape[1]/2),0),(int(img.shape[1]/2),img.shape[0]),(0,255,0),1)
    img = cv2.line(img,(0,int(img.shape[0]/2)),(img.shape[1],int(img.shape[0]/2)),(0,255,0),1)


    lower_hsv_gate = np.array([ilowH_gate, ilowS_gate, ilowV_gate])
    higher_hsv_gate = np.array([ihighH_gate, ihighS_gate, ihighV_gate])

    mask_gate=cv2.inRange(hsv, lower_hsv_gate, higher_hsv_gate)

        
    contours_gate, hierarchy_gate = cv2.findContours(dilated , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic_gate, contour_gate in enumerate(contours_gate):
        area_gate = cv2.contourArea(contour_gate)
        if (area > 3000):
            x_gate, y_gate, w_gate, h_gate = cv2.boundingRect(contour_gate)
            ccx_gate = x_gate + (w_gate/2)
            ccy_gate = y_gate + (h_gate/2)

            rect_gate = cv2.minAreaRect(contour_gate)
            box_gate = cv2.boxPoints(rect_gate)
            box_gate = np.int0(box_gate)
            img = cv2.circle(frame2, (ccx_gate, ccy_gate), 3, (255, 0, 0), -1)
            img = cv2.drawContours(frame2, [box_gate], 0, (0, 255, 0), 2)
            x_output = x_pid(ccx_gate)
            y_output = y_pid(ccy_gate)
            


    '''# Detecting objects
    blob = cv2.dnn.blobFromImage(img, 0.00392, (214, 214), (0, 0, 0), True, crop=False)

    net.setInput(blob)
    outs = net.forward(output_layers)

    # Showing informations on the screen
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.70:
                # Object detected
                #print(class_id)
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.2, 0.1)
    #print(indexes)
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0,0,255), 2)
            center=[int((x+x+w)/2),int((y+y+h)/2)]
            cv2.putText(img, label, (x, y + 30), font, 3, (0,255,0), 2)
            cv2.circle(img, (center[0], center[1]), 4, (255, 0, 0), 2)
            cx=center[0]
            cy=center[1]
            x_output = x_pid(cx)
            y_output = y_pid(cy)

        else:
            pwn_lr=1500
            pwm_ud=1500
            print('No Object Detected')
            pwm_yaw=1600
            #spin round to search object'''

    
            

    cv2.imshow("Image", img)
    pub_lr.publish(pwm_lr)
    pub_ud.publish(pwm_ud)
    rate.sleep()

    print("lpwm: ", pwm_x_1)
    print("rpwm: ", pwm_x_2)
    print("upwm: ". pwm_y)

    key = cv2.waitKey(1)
    if k==27:
        break
cap.release()
cv2.destroyAllWindows()
