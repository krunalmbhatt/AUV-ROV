import cv2
import serial

try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
except Exception as e:
    try:
        ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    except Exception as e:
        ser = serial.Serial('/dev/ttyACM2', 9600, timeout=1)
   
import math
img=cv2.imread('rov.png')
width=720
height=520
img=cv2.resize(img,(500,300),interpolation=cv2.INTER_AREA)
def nothing(x):
    pass
cv2.imshow('screen',img)
font = cv2.FONT_HERSHEY_SIMPLEX
#logitech=cv2.VideoCapture(1)
#gopro=cv2.VideoCapture(2)
#img=cv2.circle(img,(256,256),251,(0,0,255),5)
nokey_color=(100,100,100)
onkey_color=(0,0,255)
color_w=nokey_color
color_s=nokey_color
color_a=nokey_color
color_d=nokey_color
color_q=nokey_color
color_e=nokey_color
color_k=nokey_color
color_i=nokey_color

def Rand(start, end, num):
    res = []
 
    for j in range(num):
        res.append(random.randint(start, end))
 
    return res

while True:
   
    cv2.imshow('screen',img)

    #_,img1=logitech.read()
    #_,img2=gopro.read()

    #cv2.imshow('gopro',img2)
    #cv2.imshow('logiech',img1)
    key=cv2.waitKey(100)
    try:
        print(chr(key))
       
        if chr(key)=='w':
            print('Forward')
            ser.write(b'w')
            color_w=onkey_color        

        if chr(key)=='a':
            print("Lateral Left")
            ser.write(b'a')
            color_a=onkey_color

        if chr(key)=='s':
            print('Backward')
            ser.write(b's')
            color_s=onkey_color

        if chr(key)=='d':
            ser.write(b'd')
            print('Lateral Right')
            color_d=onkey_color

        if chr(key)=='q':
            print('Turn left')
            ser.write(b'q')
            color_q=onkey_color
        if chr(key)=='e':
            print('Turn Right')
            #ser.write(b'e')
            color_e=onkey_color
        if chr(key)=='i':
            #ser.write(b'i')
            print('Up')
            color_i=onkey_color
        if chr(key)=='k':
            print('Down')
            #ser.write(b'k')
            color_k=onkey_color
           
        if chr(key)=='K':
            print("Killed")
            break
       
       
       
    except Exception as e:
        print("No Key Pressed")
        #print(e)
        color_w=nokey_color
        color_s=nokey_color
        color_a=nokey_color
        color_d=nokey_color
        color_q=nokey_color
        color_e=nokey_color
        color_i=nokey_color
        color_k=nokey_color      
    img=cv2.circle(img,(280,65),20,color_w,3)
    img=cv2.circle(img,(230,65),20,color_q,3)
    img=cv2.circle(img,(330,65),20,color_e,3)
    img=cv2.circle(img,(280,115),20,color_s,3)
    img=cv2.circle(img,(230,115),20,color_a,3)
    img=cv2.circle(img,(330,115),20,color_d,3)
    img=cv2.circle(img,(430,65),20,color_i,3)
    img=cv2.circle(img,(430,115),20,color_k,3)
   
    img = cv2.putText(img, 'W', (268,77), font,
                   1, color_w, 1, cv2.LINE_AA)
    img = cv2.putText(img, 'Q', (220,77), font,
                   1, color_q, 1, cv2.LINE_AA)
    img = cv2.putText(img, 'E', (320,77), font,
                   1, color_e, 1, cv2.LINE_AA)
    img = cv2.putText(img, 'S', (270,124), font,
                   1, color_s, 1, cv2.LINE_AA)
    img = cv2.putText(img, 'A', (220,124), font,
                   1, color_a, 1, cv2.LINE_AA)
    img = cv2.putText(img, 'D', (320,124), font,
                   1, color_d, 1, cv2.LINE_AA)
    img = cv2.putText(img, 'I', (427,77), font,
                   1, color_i, 1, cv2.LINE_AA)
    img = cv2.putText(img, 'K', (420,124), font,
                   1, color_k, 1, cv2.LINE_AA)

    img = cv2.putText(img, 'Press K to Quit', (20,20), font,
                   0.6, (100,100,100), 1, cv2.LINE_AA)
    '''
    img = cv2.putText(img, 'YAW = 2.6', (20,60), font,
                   0.6, (100,100,100), 1, cv2.LINE_AA)
    img = cv2.putText(img, 'PITCH = 0.3', (20,80), font,
                   0.6, (100,100,100), 1, cv2.LINE_AA)
    img = cv2.putText(img, 'ROLL = 1.4', (20,100), font,
                   0.6, (100,100,100), 1, cv2.LINE_AA)
    img = cv2.putText(img, 'DEPTH = 1.24m', (20,120), font,
                   0.6, (100,100,100), 1, cv2.LINE_AA)'''
    color_w=nokey_color
    color_s=nokey_color
    color_a=nokey_color
    color_d=nokey_color
    color_q=nokey_color
    color_e=nokey_color
    color_k=nokey_color
    color_i=nokey_color
    ser.flushInput()
    ser.flushOutput()
   
cv2.destroyAllWindows()
