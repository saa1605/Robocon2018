import cv2
import cv2.aruco as aruco
import numpy as np
import math
##import serial

cap = cv2.VideoCapture(1)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
parameters = aruco.DetectorParameters_create()

##ser = serial.Serial("COM3",9600)
##
##def grip():
##    ser.write(b'g')

def get_aruco_points(corners, index, index_ref):
    
    new_corners = np.array(corners)     #to convert a tupple into a numpy array

    edge_center_w = (new_corners[index,0,1,0] + new_corners[index,0,2,0])/2
    edge_center_h = (new_corners[index,0,1,1] + new_corners[index,0,2,1])/2

    center_w = (new_corners[index,0,0,0] + new_corners[index,0,2,0])/2
    center_h = (new_corners[index,0,0,1] + new_corners[index,0,2,1])/2

    ref1_w = (new_corners[index_ref,0,2,0] + new_corners[index_ref,0,3,0])/2
    ref1_h = (new_corners[index_ref,0,2,1] + new_corners[index_ref,0,3,1])/2

    ref2_w = (new_corners[index_ref,0,0,0] + new_corners[index_ref,0,1,0])/2
    ref2_h = (new_corners[index_ref,0,0,1] + new_corners[index_ref,0,1,1])/2

    return center_w, center_h, edge_center_w, edge_center_h, ref1_w, ref1_h, ref2_w, ref2_h

def compute_angle(corners,center_w,center_h,edge_center_w,edge_center_h,ref1_w,ref1_h,ref2_w,ref2_h):
    
    if ref2_h - ref1_h != 0:        #to prevent division by 0 condition
        angle_ref = math.atan((ref2_w - ref1_w)/(ref2_h - ref1_h))*(-180 / math.pi)

        if (ref2_w > ref1_w) and (ref2_h > ref1_h):
            angle_ref = angle_ref + 180
        elif (ref2_w < ref1_w) and (ref2_h > ref1_h):
            angle_ref = angle_ref - 180
    else:
        if ref2_w > ref1_w:
            angle_ref = 90
        else:
            angle_ref = -90

    if edge_center_h - center_h != 0:       #to prevent division by 0 condition
        angle_raw = math.atan((edge_center_w - center_w)/(edge_center_h - center_h))*(-180 / math.pi)
        
        if (edge_center_w > center_w) and (edge_center_h > center_h):
            angle_raw = angle_raw + 180
        elif (edge_center_w < center_w) and (edge_center_h > center_h):
            angle_raw = angle_raw - 180
    else:
        if edge_center_w > center_w:
            angle_raw = 90
        else:
            angle_raw = -90

    angle = angle_raw - angle_ref
    angle1, angle2 = angle, angle

    if angle < -6:  angle -= 5
    elif angle > 5:     #numpy function of range-ish
        angle1 = 3.4172 + 0.6748*angle + 5.4321
        angle2 = -0.00129821*angle*angle + 0.841176*angle - 0.417503 + 5.4321
    
    stepped_angle = int(angle/10)

    if stepped_angle < -8: stepped_angle = -8
    elif stepped_angle > 8: stepped_angle = 8

    return angle, angle1, angle2, stepped_angle
    
while 1:
    ok, frame = cap.read()

    #detectMarkers                       
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters = parameters)

    if (type(ids) == np.ndarray) and (len(ids) == 2):         #check if 2 arucos are detected
        if ids[0] == 911:       #give the arucos different indexes
            index, index_ref = 0, 1
        elif ids[1] == 911:
            index, index_ref = 1, 0
                    
        #Get the aruco points and angle computation
        center_w,center_h,edge_center_w,edge_center_h,ref1_w,ref1_h,ref2_w,ref2_h = get_aruco_points(corners,index,index_ref)  
        angle,angle1,angle2,stepped_angle = compute_angle(corners,center_w,center_h,edge_center_w,edge_center_h,ref1_w,ref1_h,ref2_w,ref2_h)

        #Grip Function
##        grip()

        #debugging artistic skills
        cv2.line(frame,(int(edge_center_w),int(edge_center_h)),(int(center_w),int(center_h)),(0,0,255),5)
        cv2.line(frame,(int(ref1_w),int(ref1_h)),(int(ref2_w),int(ref2_h)),(255,0,0),5)
        cv2.putText(frame,str(angle),(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,0,0),2,cv2.LINE_AA)
        cv2.putText(frame,str(angle1),(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,0,0),2,cv2.LINE_AA)
        cv2.putText(frame,str(angle2),(10,70),cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,0,0),2,cv2.LINE_AA)
##        cv2.putText(frame,str(stepped_angle),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,0,0),2,cv2.LINE_AA)

    frame = aruco.drawDetectedMarkers(frame, corners, ids)
            
    cv2.imshow('aruco', frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27 or k == ord('q'): break

##ser.close()    #end of pySerial
cap.release()
cv2.destroyAllWindows()
