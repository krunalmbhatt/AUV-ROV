import cv2
import numpy as np

def nothing(x):
    pass
            
cv2.namedWindow('TrackBar')
cv2.createTrackbar('edge_1','TrackBar',0,1000,nothing)
cv2.createTrackbar('edge_2','TrackBar',0,1000,nothing)

while True:
    image=cv2.imread('1.jpg')
    orig=image.copy()

    edge_1=cv2.getTrackbarPos('edge_1','TrackBar')
    edge_2=cv2.getTrackbarPos('edge_2','TrackBar')


    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, edge_1, edge_2)

    cnts,_ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in cnts:
        epsilon = 0.01*cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        cv2.drawContours(image, [approx], 0, (0,255,0), 4)

    if len(approx)==4:
        pts1 = np.float32([approx[1][0], approx[0][0], approx[2][0], approx[3][0]])
        pts2 = np.float32([[0, 0], [approx[0][0][0]-approx[1][0][0], 0], [0, approx[3][0][1]-approx[0][0][1]], [approx[0][0][0]-approx[1][0][0], approx[3][0][1]-approx[0][0][1]]])

        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(orig, matrix, (approx[0][0][0]-approx[1][0][0], approx[3][0][1]-approx[0][0][1])) 
        cv2.imshow('result',result)

    cv2.imshow("Image", image)
    cv2.imshow("Edged", edged)
    cv2.waitKey(1)
cv2.destroyAllWindows()
