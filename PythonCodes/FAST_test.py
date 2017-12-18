import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(0)

while(1):
##    # Start timer
##    timer = cv2.getTickCount()
    
    #Capture freq is 16-20    
    ok, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

##     # Calculate Frames per second (FPS)
##    fps1 = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
##    
##    # Display FPS on frame
##    cv2.putText(gray, str(int(fps1)), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    cv2.imshow('camera',gray)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 13: break
    elif k == 27:
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()

bbox = cv2.selectROI(gray)
obj = gray[int(bbox[1]):int(bbox[1]+bbox[3]), int(bbox[0]):int(bbox[0]+bbox[2])]

#hessianthreshold, nOctaves, nOctaveLayers, extended, upright
##surf = cv2.xfeatures2d.SURF_create(300,4,3,True,False)

fast = cv2.FastFeatureDetector_create()
print(fast.descriptorType())
##kp1, des1 = surf.detectAndCompute(obj,None)

kp1 = fast.detect(obj)

while(1):
    ok, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Start timer
    timer = cv2.getTickCount()

##    kp2, des2 = surf.detectAndCompute(gray,None)
##
    
    kp2,des2 = orb.detect(gray,None)
##    # FLANN parameters
##    FLANN_INDEX_KDTREE = 0
##    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
##    search_params = dict(checks=50)   # or pass empty dictionary
##     
##    flann = cv2.FlannBasedMatcher(index_params,search_params)
##     
##    matches = flann.knnMatch(des1,des2,k = 2)
##    
##    # Need to draw only good matches, so create a mask
##    matchesMask = [[0,0] for i in range(len(matches))]
##     
##    # ratio test as per Lowe's paper
##    for i,(m,n) in enumerate(matches):
##        if m.distance < 0.7*n.distance:
##            matchesMask[i]=[1,0]
##     
##    draw_params = dict(matchColor = (0,255,0),
##                       singlePointColor = (255,0,0),
##                       matchesMask = matchesMask,
##                       flags = 0)

    mask = kp2 and gray
    
    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    
    # Display FPS on frame
    cv2.putText(gray, str(int(fps)), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    img3 = cv2.drawMatchesKnn(obj,kp1,gray,kp2,matches,None,**draw_params) 
    cv2.imshow('detect', img3)

    k = cv2.waitKey(1) & 0xFF
    if k == 27: break

cap.release()
cv2.destroyAllWindows()
