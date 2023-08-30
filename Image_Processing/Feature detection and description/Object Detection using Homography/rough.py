import cv2
import numpy as np
import math

#img = cv2.imread("book.jpg", cv2.IMREAD_GRAYSCALE)  # queryiamge
    
cap = cv2.VideoCapture(0)
_,img=cap.read()
img=img[0:100,0:100]
img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#Features
sift = cv2.xfeatures2d.SIFT_create()
kp_image, desc_image = sift.detectAndCompute(img, None)

# Feature matching remember fixed steps
index_params = dict(algorithm=0, trees=5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)

while True:
    _, frame = cap.read()
    grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # trainimage
    kp_grayframe, desc_grayframe = sift.detectAndCompute(grayframe, None)
    matches = flann.knnMatch(desc_image, desc_grayframe, k=2)
    good_points = []
    for m, n in matches:
        if m.distance < 0.6 * n.distance:#change 0.6 to change accuracy
            good_points.append(m)
    # img3 = cv2.drawMatches(img, kp_image, grayframe, kp_grayframe, good_points, grayframe)
    # Homography
    if len(good_points) > 10:#change 10 to change accuracy
        query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
        matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
        matches_mask = mask.ravel().tolist()
        # Perspective transform
        h, w = img.shape
        pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, matrix)
        homography = cv2.polylines(frame, [np.int32(dst)], True, (0, 255, 0), 3)
        points=np.int32(dst)
        points=points.ravel()
        #print(points)
        x1=points[0]#top left
        y1=points[1]#top left
        x2=points[2]#bottom left
        y2=points[3]#bottom left
        x3=points[4]#bottom right
        y3=points[5]#bottom right
        x4=points[6]#top right
        y4=points[7]#top right

        x_center=int((x1+x2+x3+x4)/4)
        y_center=int((y1+y2+y3+y4)/4)

        #3D-Effect

        #vertical and horizontal angle find
        hor_line_up=[x4-x1,y4-y1]
        ver_line_left=[x2-x1,y2-y1]
        hor_line_down=[x3-x2,y3-y2]
        ver_line_right=[x4-x3,y4-y3]

        #length
        len_hor_line_up=math.sqrt(abs((hor_line_up[0])**2+(hor_line_up[1])**2))
        len_hor_line_down=math.sqrt(abs((hor_line_down[0])**2+(hor_line_down[1])**2))
        len_ver_line_left=math.sqrt(abs((ver_line_left[0])**2+(ver_line_left[1])**2))
        len_ver_line_right=math.sqrt(abs((ver_line_right[0])**2+(ver_line_right[1])**2))

        inclination_ver=((len_ver_line_left-len_ver_line_right)*360)/len_ver_line_left#|I or I|
        
        
        print("inclination_ver :",int(inclination_ver))
       
        if int(inclination_ver)>0:
            a=str(int(inclination_ver))
            string="INCLINATION = INSIDE ="+a
            cv2.putText(frame,string,(40,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
        if int(inclination_ver)<0:
            a=str(int(-inclination_ver))
            string="INCLINATION = OUTSIDE ="+a
            cv2.putText(frame,string,(40,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
            
        cv2.putText(frame,"OBJECT",(x1+10,y1+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),2)
        cv2.putText(frame,"CENTER",(x_center+10,y_center+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
        circle_draw = cv2.circle(frame,(x_center,y_center),2,(0,0,255),2)    
        cv2.imshow("Homography", homography)
    else:
        cv2.imshow("Homography", frame)
    # cv2.imshow("Image", img)
    # cv2.imshow("grayFrame", grayframe)
    # cv2.imshow("img3", img3)
    key = cv2.waitKey(25)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()
