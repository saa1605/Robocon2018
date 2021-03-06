import cv2
import numpy as np
import sys
cv2.ocl.setUseOpenCL(False)

cap = cv2.VideoCapture(0)
MIN_MATCH_COUNT = 10

while(1):
    # Start timer
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

orb = cv2.ORB_create(nfeatures = 5000)

kp1,des1 = orb.detectAndCompute(obj,None)

while(1):
    ok, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Start timer
    timer = cv2.getTickCount()

    kp2,des2 = orb.detectAndCompute(gray,None)

    # FLANN parameters
    FLANN_INDEX_LSH = 6
    index_params = dict(algorithm = FLANN_INDEX_LSH,
                        table_number = 6, #12
                        key_size = 12, #20
                        multi_probe_level = 1) # 2
    search_params = dict(checks=100)   # or pass empty dictionary
    
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    
    matches = flann.knnMatch(des1, des2, k = 2)

##    bf = cv2.BFMatcher()
##    matches = bf.knnMatch(des1,des2, k=2)
  
    # Need to draw only good matches, so create a mask
    #matchesMask = [[0,0] for i in range(len(matches))]

    good = []
    # ratio test as per Lowe's paper
    try:
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.7*n.distance:
                #matchesMask[i]=[1,0]
                good.append(m)
    except:
        continue

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w = obj.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        x_centroid = np.average(dst[:,0,0])
        y_centroid = np.average(dst[:,0,1])
        print("Centroid : (%d,%d)" % (x_centroid,y_centroid))
##        print(dst.shape)
##        print(dst)
        
        gray = cv2.polylines(gray,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        gray = cv2.circle(gray,(x_centroid,y_centroid),10,255,-1,cv2.LINE_AA)

    else:
##        print ("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        matchesMask = None
    
    draw_params = dict(matchColor = (0,255,0),
                       singlePointColor = (255,0,0),
                       matchesMask = matchesMask,
                       flags = 0)
    
    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    
    # Display FPS on frame
    cv2.putText(gray, str(int(fps)), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

##    img = cv2.drawMatchesKnn(obj,kp1,gray,kp2,matches,None,**draw_params)
    img = cv2.drawMatches(obj,kp1,gray,kp2,good,None,**draw_params)
    cv2.imshow('detect', img)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27: break

cap.release()
cv2.destroyAllWindows()
