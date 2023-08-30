import cv2
import numpy as np

def trim(frame):
    #crop top
    if not np.sum(frame[0]):
        return trim(frame[1:])
    #crop top
    if not np.sum(frame[-1]):
        return trim(frame[:-2])
    #crop top
    if not np.sum(frame[:,0]):
        return trim(frame[:,1:])
    #crop top
    if not np.sum(frame[:,-1]):
        return trim(frame[:,:-2])
    return frame


cap=cv2.VideoCapture(1)
x=0
while(True):
    if x==0:
        _,img1=cap.read()
        img1_gray=cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
        #cv2.waitKey(200)            
        _,img2=cap.read()
        img2_gray=cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        x=69420

    if x==69:
        img1=stich
        #cv2.waitKey(200)
        img1_gray=cv2.cvtColor(stich,cv2.COLOR_BGR2GRAY)
        #cv2.waitKey(100)
        
        _,img2=cap.read()
        img2_gray=cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

    sift = cv2.xfeatures2d.SIFT_create()
    # find key points
    kp1, des1 = sift.detectAndCompute(img1_gray,None)
    kp2, des2 = sift.detectAndCompute(img2_gray,None)


    #cv2.imshow('original_image_left_keypoints',cv2.drawKeypoints(img_,kp1,None))

    #FLANN_INDEX_KDTREE = 0
    #index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    #search_params = dict(checks = 50)
    #match = cv2.FlannBasedMatcher(index_params, search_params)
    match = cv2.BFMatcher()
    matches = match.knnMatch(des2,des1,k=2)

    good = []
    for m,n in matches:
        #if m.distance < 0.5*n.distance:
        good.append(m)

    draw_params = dict(matchColor=(0,255,0),
                           singlePointColor=None,
                           flags=2)

    img3 = cv2.drawMatches(img2,kp2,img1,kp1,good,None,**draw_params)
    #cv2.imshow("original_image_drawMatches.jpg", img3)

    MIN_MATCH_COUNT = 0
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([ kp2[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp1[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        h,w = img1_gray.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts, M)
        img22 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        #cv2.imshow("original_image_overlapping.jpg", img2)
    else:
        print("Not enough matches are found")

    dst = cv2.warpPerspective(img2,M,(img1.shape[1] + img2.shape[1], img1.shape[0]))
    dst[0:img1.shape[0],0:img1.shape[1]] = img1
    #cv2.imshow("original_image_stitched.jpg", dst)

    stich=trim(dst)

    cv2.imshow("original_image_stitched_crop.jpg",stich )
    k=cv2.waitKey(100)
    x=69
    if k==27:
        cv2.imwrite('panorama1.jpeg',stich)
        break
cap.release()
cv2.destroyAllWindows
    
