import numpy as np
import cv2
import cv2.aruco as aruco
counter = 0

cap = cv2.VideoCapture(1)

while(True):
    # Capture frame-by-frame
    ret, gray = cap.read()
    #print(frame.shape) #480x640
    #Our operations on the frame come here
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    parameters =  aruco.DetectorParameters_create()

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    new_corners = np.array(corners)
    print(new_corners.shape)
    if new_corners.shape == (1,1,4,2):
        x_redSquare = new_corners[0,0,0,0]
        y_redSquare = new_corners[0,0,0,1]
        x_oppRedSquare = new_corners[0,0,2,0]
        y_oppRedSquare = new_corners[0,0,2,1]
        cv2.line(gray,(x_redSquare,y_redSquare),(x_oppRedSquare,y_oppRedSquare),(0,0,255),5)
    else:
        cv2.putText(gray,'FUCK',(500,500), cv2.FONT_HERSHEY_SIMPLEX, 5,(0,0,155),10,cv2.LINE_AA)        

    # print((x_redSquare,y_redSquare))
    # It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs

    gray = aruco.drawDetectedMarkers(gray, corners,ids)
    #print(rejectedImgPoints)
    #`Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# import numpy as np
# import cv2
# import cv2.aruco as aruco
#
#
# '''
#     drawMarker(...)
#         drawMarker(dictionary, id, sidePixels[, img[, borderBits]]) -> img
# '''
#
# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
# print(aruco_dict)
# # second parameter is id number
# # last parameter is total image size
# img = aruco.drawMarker(aruco_dict, 2, 700)
# cv2.imwrite("test_marker.jpg", img)
#
# cv2.imshow('frame',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()