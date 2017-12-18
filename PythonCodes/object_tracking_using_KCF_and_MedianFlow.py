import cv2
import sys

tracker1 = cv2.TrackerKCF_create()
tracker2 = cv2.TrackerMedianFlow_create()

cap = cv2.VideoCapture(0) 

# Exit if video not opened.
if not cap.isOpened():
    print ('Could not capture video')
    sys.exit()

while(1):
    ok, frame = cap.read()
    if not ok:
        print ('Could not read frame')
        sys.exit()
    cv2.imshow('camera',frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 13:
        break

bbox = cv2.selectROI(frame)

# Initialize tracker with first frame and bounding box
ok = tracker1.init(frame, bbox) and tracker2.init(frame, bbox)

while (True):
    ok, frame = cap.read()
    if not ok:
        break
     
    # Start timer
    timer = cv2.getTickCount()

    # Update tracker
    ok, bbox = tracker1.update(frame)
    ok, bbox1 = tracker2.update(frame)
    import operator
    tuple(map(operator.add, bbox, bbox1))
##    bbox[0], bbox[1], bbox[2], bbox[3] = int(bbox[0] + bbox2[0]), int(bbox[1] + bbox2[1]),int(bbox[2] + bbox2[2]),int(bbox[3] + bbox2[3])
    
    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    else :
        # Tracking failure
        cv2.putText(frame, "Tracking Failure Detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)

    # Interaction with atmega
    myCentroid = ( int(bbox[0])+int(bbox[0]+bbox[2]) )/ 2
    _,w,h = frame.shape[::-1]
    
    if myCentroid > (w/2)+10:
        
##        serial.write('r')
        cv2.putText(frame, "Left", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    elif myCentroid < (w/2)-10:
##        serial.write('l')
        cv2.putText(frame, "Right", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    else:
##        serial.write('c')
        cv2.putText(frame, "Center", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)
    
    # Display FPS on frame
    cv2.putText(frame, str(int(fps)), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 2)

    # Display result
    cv2.imshow('Tracking', frame)

    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27 : break

cap.release()
cv2.destroyAllWindows()
