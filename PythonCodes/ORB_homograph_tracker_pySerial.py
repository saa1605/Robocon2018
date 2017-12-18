import cv2
import numpy as np
import serial
import sys
cv2.ocl.setUseOpenCL(False)

cap = cv2.VideoCapture(0)
MIN_MATCH_COUNT = 10
count = 100
threshold = 10

while(1):
    # Start timer
    timer = cv2.getTickCount()

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
        print("EXIT")
        ser.close()
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()

# Start of Obj Detection
bbox = cv2.selectROI(gray)
obj = gray[int(bbox[1]):int(bbox[1]+bbox[3]), int(bbox[0]):int(bbox[0]+bbox[2])]

orb = cv2.ORB_create(nfeatures = 4000)
kp1,des1 = orb.detectAndCompute(obj,None)

while(count):
    ok, frame = cap.read()
    frame_copy = frame.copy()
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
        global dst
        dst = cv2.perspectiveTransform(pts,M)
##        x_centroid = np.average(dst[:,0,0])
##        y_centroid = np.average(dst[:,0,1])
##        print("Centroid : (%d,%d)" % (x_centroid,y_centroid))
        count = count - 1
 
        gray = cv2.polylines(gray,[np.int32(dst)],True,255,3, cv2.LINE_AA)
##        gray = cv2.circle(gray,(x_centroid,y_centroid),10,255,-1,cv2.LINE_AA)

    else:
        #print ("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
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
    if k == 27:
        ##ser.close()
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()

print("Obj Detected")

# End of Obj Detection
#-------------------------------------------------------------------------------------------
# Start of Trackers KCF and MEDIANFLOW

ser = serial.Serial("COM3",9600)
tracker_KCF = cv2.TrackerKCF_create()
tracker_MEDIANFLOW = cv2.TrackerMedianFlow_create()
bbox = np.array(bbox)
bbox[0] = dst[0,0,0]
bbox[1] = dst[0,0,1]
bbox[2] = dst[2,0,0] - bbox[0]
bbox[3] = dst[2,0,1] - bbox[1]
bbox = tuple(bbox)

# Initialize tracker with first frame and bounding box
ok = tracker_KCF.init(frame_copy, bbox) and tracker_MEDIANFLOW.init(frame_copy, bbox)
print("Start Tracking")

while (True):
    ok, frame = cap.read()
    if not ok:
        break
     
    # Start timer
    timer = cv2.getTickCount()

    # Update tracker
    ok, bbox1 = tracker_KCF.update(frame)
    ok, bbox2 = tracker_MEDIANFLOW.update(frame)
    import operator
    tuple(map(operator.add, bbox, bbox1))
    
    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox1[0]), int(bbox1[1]))
        p2 = (int(bbox1[0] + bbox1[2]), int(bbox1[1] + bbox1[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    else :
        # Tracking failure
        cv2.putText(frame, "Tracking Failure Detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)

    # pySerial Commands
    myCentroid = (int(bbox1[0])+int(bbox1[0]+bbox1[2]))/ 2
    _,w,h = frame.shape[::-1]
    
    if myCentroid > (w/2) + threshold:
        ser.write(b'r')
        cv2.putText(frame, "Left", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    elif myCentroid < (w/2) - threshold:
        ser.write(b'l')
        cv2.putText(frame, "Right", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    else:
        ser.write(b'c')
        cv2.putText(frame, "Center", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)
    
    # Display FPS on frame
    cv2.putText(frame, str(int(fps)), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    # Display result
    cv2.imshow('Tracking', frame)

    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27 : break

print("EXIT")
ser.close()
cap.release()
cv2.destroyAllWindows()
