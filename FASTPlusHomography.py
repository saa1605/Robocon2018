import cv2
import numpy as np
import sys
cv2.ocl.setUseOpenCL(False)

cap = cv2.VideoCapture(0)

while(1):
#   Start timer
    timer = cv2.getTickCount()

    #Capture freq is 16-20    
    ok, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Calculate Frames per second (FPS)
    fps1 = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    # Display FPS on frame
    cv2.putText(gray, str(int(fps1)), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

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
# fast = cv2.xfeatures2d.SURF_create(300,4,3,True,False)

fast = cv2.FastFeatureDetector_create(15)
# orb= cv2.ORB_create(nfeatures=1000)

# kp1,des1 =orb.detectAndCompute(obj,None)
brief = cv2.xfeatures2d.BriefDescriptorExtractor_create(16)
kp1 = fast.detect(obj)
kp1, des1 = brief.compute(obj,kp1)
while(1):
    ok, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Start timer
    timer = cv2.getTickCount()

    # kp2,des2 = orb.detectAndCompute(gray,None)
    # # des2 = fast.compute(gray,kp2)
    kp2 = fast.detect(gray)
    kp2, des2 = brief.compute(gray,kp2)
    # FLANN parameters
    FLANN_INDEX_LSH = 6
    index_params = dict(algorithm = FLANN_INDEX_LSH, table_number = 6, key_size = 12, multi_probe_level = 1)
    search_params = dict(checks=50)   # or pass empty dictionary

    flann = cv2.FlannBasedMatcher(index_params,search_params)

    matches = flann.knnMatch(des1,des2,k=2)
    
    # bf = cv2.BFMatcher()
    # matches = bf.knnMatch(des1,des2, k=2)

    # Need to draw only good matches, so create a mask
    matchesMask = [[0,0] for i in range(len(matches))]
     
    # ratio test as per Lowe's paper
    MIN_MATCH_COUNT = 10
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
            
            
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w = obj.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        print dst.shape
        gray = cv2.polylines(gray,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        x_centroid = np.average(dst[:,0,0])
        y_centroid = np.average(dst[:,0,1])
        print x_centroid
        print y_centroid
    else:
        print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
        matchesMask = None
    # MIN_MATCH_COUNT = 10
    # if len(good)>MIN_MATCH_COUNT:
    #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    #     dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)    
    
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)

 
    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    print(len(kp2))
    # Display FPS on frame
    cv2.putText(gray, str(int(fps)), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    img3 = cv2.drawMatches(obj,kp1,gray,kp2,good,None,**draw_params)
    
    cv2.imshow('detect', img3)

    k = cv2.waitKey(1) & 0xFF
    if k == 27: break

cap.release()
cv2.destroyAllWindows()
