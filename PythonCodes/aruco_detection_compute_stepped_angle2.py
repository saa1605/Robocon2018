<<<<<<< Local Changes
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import serial

cap = cv2.VideoCapture(0)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
parameters = aruco.DetectorParameters_create()
step = 10

##ser = serial.Serial("COM3",9600)
##
##def grip():
##    ser.write(b'g')

def get_aruco_points(corners,index):
    
    new_corners = np.array(corners)

    edge_center_w = (new_corners[index,0,0,0] + new_corners[index,0,1,0])/2
    edge_center_h = (new_corners[index,0,0,1] + new_corners[index,0,1,1])/2

    center_w = (new_corners[index,0,0,0] + new_corners[index,0,2,0])/2
    center_h = (new_corners[index,0,0,1] + new_corners[index,0,2,1])/2

    return center_w,center_h,edge_center_w,edge_center_h

def compute_angle(center_w,center_h,edge_center_w,edge_center_h):

    if edge_center_h - center_h != 0:
        angle = math.atan((edge_center_w - center_w)/(edge_center_h - center_h))*(-180/math.pi)
        
        if (edge_center_w > center_w) and (edge_center_h > center_h):
            angle = angle + 180
        elif (edge_center_w < center_w) and (edge_center_h > center_h):
            angle = angle - 180
    else:
        if edge_center_w > center_w:
            angle = 90
        else:
            angle = -90
            
    stepped_angle = int(angle/step)

    return angle,stepped_angle

    
while(1):
    ok, frame = cap.read()

    #detectMarkers                       
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    index = 0

    if type(ids) == np.ndarray:         #check if any aruco is detected
        while(index < len(ids)):        #find the required aruco id out of all
            if ids[index] == 911:

                #Get corners of aruco points and angle computation 
                center_w, center_h, edge_center_w, edge_center_h = get_aruco_points(corners,index)  
                angle,stepped_angle = compute_angle(center_w,center_h,edge_center_w,edge_center_h)

                #Grip Function
                #grip()
                
                #debugging artistic skills
                cv2.line(frame,(int(edge_center_w),int(edge_center_h)),(int(center_w),int(center_h)),(0,0,255),5)
                cv2.putText(frame,str(angle),(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,0,0),2,cv2.LINE_AA)
                cv2.putText(frame,str(stepped_angle),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,0,0),2,cv2.LINE_AA)

            index = index + 1
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('aruco',frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27 or k == ord('q'): break

##ser.close()    
cap.release()
cv2.destroyAllWindows()
=======
>>>>>>> External Changes
