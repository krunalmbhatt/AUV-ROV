import cv2
import numpy as np


def empty(value):
    try:
        value = float(value)
    except ValueError:
        pass
    return bool(value)


def callback(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow('image')





ilowH = 0
ihighH = 255
ilowS = 0
ihighS = 255
ilowV = 0
ihighV = 255

cv2.createTrackbar('RlowH','image',ilowH,255,callback)
cv2.createTrackbar('RhighH','image',ihighH,255,callback)

cv2.createTrackbar('RlowS','image',ilowS,255,callback)
cv2.createTrackbar('RhighS','image',ihighS,255,callback)

cv2.createTrackbar('RlowV','image',ilowV,255,callback)
cv2.createTrackbar('RhighV','image',ihighV,255,callback)




cv2.createTrackbar('GlowH','image',ilowH,255,callback)
cv2.createTrackbar('GhighH','image',ihighH,255,callback)
cv2.createTrackbar('GlowS','image',ilowS,255,callback)
cv2.createTrackbar('GhighS','image',ihighS,255,callback)
cv2.createTrackbar('GlowV','image',ilowV,255,callback)
cv2.createTrackbar('GhighV','image',ihighV,255,callback)

while (1):
    _, frame = cap.read()
    #frame= cv2.imread("r&g.PNG")
    frame = cv2.resize(frame, (420, 420))

    frame2 = frame
    frame3 = frame
    frame4 = frame
    RilowH = cv2.getTrackbarPos('RlowH', 'image')
    RihighH = cv2.getTrackbarPos('RhighH', 'image')
    RilowS = cv2.getTrackbarPos('RlowS', 'image')
    RihighS = cv2.getTrackbarPos('RhighS', 'image')
    RilowV = cv2.getTrackbarPos('RlowV', 'image')
    RihighV = cv2.getTrackbarPos('RhighV', 'image')




    GilowH = cv2.getTrackbarPos('GlowH', 'image')
    GihighH = cv2.getTrackbarPos('GhighH', 'image')
    GilowS = cv2.getTrackbarPos('GlowS', 'image')
    GihighS = cv2.getTrackbarPos('GhighS', 'image')
    GilowV = cv2.getTrackbarPos('GlowV', 'image')
    GihighV = cv2.getTrackbarPos('GhighV', 'image')


    Rhsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    Ghsv = cv2.cvtColor(frame3, cv2.COLOR_BGR2LAB)
    #Rlower_hsv = np.array([125, 46, 147])
    #Rhigher_hsv = np.array([255, 100, 200])


    Rlower_hsv = np.array([RilowH, RilowS, RilowV])
    Rhigher_hsv = np.array([RihighH, RihighS, RihighV])


    Glower_hsv = np.array([GilowH, GilowS, GilowV])
    Ghigher_hsv = np.array([GihighH, GihighS, GihighV])
    """LAB = 173,178,110,115,113,131"""
    #Glower_hsv = np.array([173, 110, 113])
    #Ghigher_hsv = np.array([178, 115, 131])

    Rmask = cv2.inRange(Rhsv, Rlower_hsv, Rhigher_hsv)
    
    Gmask = cv2.inRange(Ghsv, Glower_hsv, Ghigher_hsv)
    
    frame = np.array(255 * (frame / 255) ** 0.6, dtype='uint8')
    frame3 = np.array(255 * (frame3 / 255) ** 0.6, dtype='uint8')




    dil_kernel = np.ones((21,21), np.uint8)
    ero_kernal = np.ones((5,5), np.uint8)
    Rerode = cv2.erode(Rmask, ero_kernal)
    Rdilated = cv2.dilate(Rerode,dil_kernel)
    cv2.imshow("rdilate", Rdilated)
    #Rdilated = cv2.bitwise_and(frame2, frame2, mask=Rmask)

    Gdilated = cv2.dilate(Gmask,dil_kernel)
    cv2.imshow("gdilate", Gdilated)
    #dilated = cv2.add(Rdilated, Gdilated)
    Rcontours, hierarchy = cv2.findContours(Rdilated , cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    x1= 0
    x2= 0
    x3= 0
    x4= 0
    y1= 0
    y2= 0
    y3= 0
    y4= 0
    cx= 0
    cy= 0
    
    
    for pic, Rcontour in enumerate(Rcontours):
        Rarea = cv2.contourArea(Rcontour)
        if (Rarea > 700):
            #x, y, w, h = cv2.boundingRect(contour)
            #cx = x + (w/2)
            #cy = y + (h/2)
            
            Rrect = cv2.minAreaRect(Rcontour)
            Rbox = cv2.boxPoints(Rrect)
            Rbox = np.int0(Rbox)
            x1, y1 = Rbox[0]
            x2, y2 = Rbox[1]
            x3, y3 = Rbox[2]
            x4, y4 = Rbox[3]
            cx = int((x1 + x3)/2)
            cy = int((y1 + y3)/2)
            Rimg = cv2.circle(frame2,(cx, cy), 5, (0,255,0), -1)
            Rimg = cv2.drawContours(Rimg, [Rbox], 0, (0, 0, 255), 2)
            #cv2.putText(img, "RED color", (50,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

            #print(box)
            #print(x1, y1, x2, y2, x3, y3, x4, y4)
           #  cv2.putText(img, "cx",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 0))

            #res = cv2.bitwise_and(frame,frame, mask=mask)

    a1= 0
    a2= 0
    a3= 0
    a4= 0
    b1= 0
    b2= 0
    b3= 0
    b4= 0
    ca= 0
    cb= 0
    Gcontours, hierarchy = cv2.findContours(Gdilated , cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for Gpic, Gcontour in enumerate(Gcontours):
        Garea = cv2.contourArea(Gcontour)
        if (Garea > 700):
            #x, y, w, h = cv2.boundingRect(contour)
            #cx = x + (w/2)
            #cy = y + (h/2)
            
            Grect = cv2.minAreaRect(Gcontour)
            Gbox = cv2.boxPoints(Grect)
            Gbox = np.int0(Gbox)
            a1, b1 = Gbox[0]
            a2, b2 = Gbox[1]
            a3, b3 = Gbox[2]
            a4, b4 = Gbox[3]
            ca = int((a1 + a3)/2)
            cb = int((b1 + b3)/2)
            Gimg = cv2.circle(frame2,(ca, cb), 5, (0,0,255), -1)
            Gimg = cv2.drawContours(Gimg, [Gbox], 0, (0, 255, 0), 2)
            #cv2.putText(img, "RED color", (50,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))



    frame2 = cv2.line(frame2, (cx, cy), (ca, cb), (255, 0, 0), 3)
    mx = int((cx + ca)/2)
    my = int((cy + cb)/2)
    frame2 = cv2.circle(frame2,(mx, my), 7, (255,0,0), -1)
    print(mx, my)

            #print(box)
            #print(x1, y1, x2, y2, x3, y3, x4, y4)
           #  cv2.putText(img, "cx",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 0))

            #res = cv2.bitwise_and(frame,frame, mask=mask)


    frame = cv2.bitwise_and(Rhsv, Rhsv, mask=Rmask)
    frame3 = cv2.bitwise_and(Ghsv, Ghsv, mask = Gmask)
    ret,Rthrshed = cv2.threshold(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    Rcontours,Rhier = cv2.findContours(Rthrshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)





    ret,Gthrshed = cv2.threshold(cv2.cvtColor(frame3,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    Gcontours,Ghier = cv2.findContours(Gthrshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    #for cnt in contours:
       # area = cv2.contourArea(cnt)
        #if area > 1000:
           # cv2.putText(frame, "Green", (10,80),cv2.FONT_HERSHEY_SIMPLEX, 1, 1)
    #cv2.rectangle(frame2,(20,20),(600,400),(0,255,255),2)
    cv2.imshow('Rframe',frame2)
    
    #cv2.imshow('Gframe', frame4)
    #cv2.imshow()




    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break




cap.release()
cv2.destroyAllWindows()
