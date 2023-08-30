while True:
    frame=cv2.resize(frame,(1920,1080),interpolation = cv2.INTER_AREA)
    #frame=cv2.GaussianBlur(frame,(15,15),2)
    #frame=cv2.bilateralFilter(frame,15,100,100)
    
    #hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2LAB)#YCrCb is also good
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)#YCrCb is also good
    
    #blur=cv2.blur(hsv,(2,2))
    blur=hsv
    


    H_Min=cv2.getTrackbarPos('H_Min','TrackBar')
    H_Max=cv2.getTrackbarPos('H_Max','TrackBar')
    S_Min=cv2.getTrackbarPos('S_Min','TrackBar')
    S_Max=cv2.getTrackbarPos('S_Max','TrackBar')
    V_Min=cv2.getTrackbarPos('V_Min','TrackBar')
    V_Max=cv2.getTrackbarPos('V_Max','TrackBar')

	
    lower=np.array([H_Min,S_Min,V_Min])
    upper=np.array([H_Max,S_Max,V_Max])
    mask=cv2.inRange(blur,lower,upper)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)#(image,contour retrival mode,contour approximation methord)
    for i in contours:
        area = cv2.contourArea(i)
        area_tbar=cv2.getTrackbarPos('AREA','TrackBar')

        if area>=area_tbar:
            M=cv2.moments(i)
            if M["m00"]!=0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])                
            cv2.drawContours(frame, i, -1, (0, 255, 0),2)#(image on which to draw contour,contours,index of contour(-1 for all),color,thickness)            
            cv2.putText(frame,"CENTER",(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
            circle_draw = cv2.circle(frame,(int(cx),int(cy)),2,(0,0,255),2)
    
    cv2.imshow("Mask", mask)
    cv2.imshow("frame",frame)
    key=cv2.waitKey(1)
    if key==27:
        break
        
#cap.release()
cv2.destroyAllWindows()
