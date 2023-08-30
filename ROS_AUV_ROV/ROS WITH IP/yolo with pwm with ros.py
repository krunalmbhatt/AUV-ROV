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
net = cv2.dnn.readNet("custom-yolov4-tiny-detector_final.weights", "custom-yolov4-tiny-detector.cfg")
#net = cv2.dnn.readNet("yolov3_training_last.weights", "yolov3_testing.cfg")
# Name custom object
classes = ["buoy"]
# Images path
#images_path = glob.glob(r"D:\Pysource\Youtube\2020\105) Train Yolo google cloud\dataset\*.jpg")
cap=cv2.VideoCapture(0)

layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
b=0

_,img=cap.read()
center_frame=[int(img.shape[0]/2),int(img.shape[1]/2)]

while not rospy.is_shotdown():
    pwm_lr = 1500
    pwm_ud = 1500
    pid_y = 0
    pid_x = 0

    b=b+1
    _,img = cap.read()

    #img = cv2.resize(img, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape
    img = cv2.line(img,(int(img.shape[1]/2),0),(int(img.shape[1]/2),img.shape[0]),(0,255,0),1)
    img = cv2.line(img,(0,int(img.shape[0]/2)),(img.shape[1],int(img.shape[0]/2)),(0,255,0),1)
    
    # Detecting objects
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
            pid_x = int(x_output)
            pid_y = int(y_output)
            pwm_lr = 1500 + pid_x
            pwm_ud = 1500 + pid_y
        else:
            pwn_lr=1500
            pwm_ud=1500
            print('No Object Detected')
            #spin round to search object
            

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
