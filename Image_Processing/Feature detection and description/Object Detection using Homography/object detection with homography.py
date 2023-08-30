import cv2
import numpy as np
import math

img = cv2.imread("buoy.jpg", cv2.IMREAD_GRAYSCALE)  # queryiamge
    
cap = cv2.VideoCapture('test video.mp4')
# Features
sift = cv2.xfeatures2d.SIFT_create()
kp_image, desc_image = sift.detectAndCompute(img, None)

# Feature matching

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
    try:
        if len(good_points) > 1:#change 10 to change accuracy
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
            '''
            #vertical and horizontal angle find
            hor_line=[x4-x1,y4-y1]
            ver_line=[x2-x1,y2-y1]

            mag_hor=math.sqrt((hor_line[0]*hor_line[0])+(hor_line[1]*hor_line[1]))
            mag_ver=math.sqrt((ver_line[0]*ver_line[0])+(ver_line[1]*ver_line[1]))

            unit_hor=[hor_line[0]/mag_hor,hor_line[1]/mag_hor]
            unit_ver=[ver_line[0]/mag_ver,ver_line[1]/mag_ver]
            
            
            yh=unit_hor[1]
            xh=unit_hor[0]

            yv=unit_ver[1]
            xv=unit_ver[0]
            
            hor_angle=math.atan(yh/xh)*180/3.14
            ver_angle=math.atan(xv/yv)*180/3.14

            print(hor_angle,'      ',ver_angle)'''

                            
            cv2.putText(frame,"OBJECT",(x1+10,y1+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),2)
            cv2.putText(frame,"CENTER",(x_center+10,y_center+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
            circle_draw = cv2.circle(frame,(x_center,y_center),2,(0,0,255),2)    
            cv2.imshow("Homography", homography)
        else:
            cv2.imshow("Homography", frame)
    except Exception as e:
        pass
        # cv2.imshow("Image", img)
        # cv2.imshow("grayFrame", grayframe)
        # cv2.imshow("img3", img3)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()
