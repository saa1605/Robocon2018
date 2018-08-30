import cv2
import numpy as np
import math

newIdc = 0

cap = cv2.VideoCapture(1)
i = 0

def maxContour(cnts):
    idc = 0
    max_area = 0
    counter = 0
    for n in cnts:
        a = cv2.contourArea(n)
        if a>max_area:
            max_area = a
            idc = counter
        counter+=1

    return idc,max_area


while True:
    ret, frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1)  == 13:
        break
bboxRed = cv2.selectROI(frame)
redObj = frame_hsv[int(bboxRed[1]):int(bboxRed[1]+bboxRed[3]), int(bboxRed[0]):int(bboxRed[0]+bboxRed[2])]

bboxBlue = cv2.selectROI(frame)
blueObj = frame_hsv[int(bboxBlue[1]):int(bboxBlue[1]+bboxBlue[3]), int(bboxBlue[0]):int(bboxBlue[0]+bboxBlue[2])]

hR, sR, vR = np.median(redObj[:,:,0]), np.median(redObj[:,:,1]), np.median(redObj[:,:,2])
lower_rangeRed = np.array([hR-5, 0, vR-70])
higher_rangeRed = np.array([hR+5, sR+50, vR+70])

hB, sB, vB = np.median(blueObj[:,:,0]), np.median(blueObj[:,:,1]), np.median(blueObj[:,:,2])
lower_rangeBlue = np.array([hB-5, sB-50, vB-70])
higher_rangeBlue = np.array([hB+5, sB+50, vB+70])

while True:
    ret, frame = cap.read()
    frame_h = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    maskRed = cv2.inRange(frame_h, lower_rangeRed, higher_rangeRed)
    maskBlue = cv2.inRange(frame_h, lower_rangeBlue, higher_rangeBlue)

    _, contoursRed, hR = cv2.findContours(maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    _, contoursBlue, hB = cv2.findContours(maskBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    redIndex,redValue = maxContour(contoursRed)
    blueIndex,blueValue = maxContour(contoursBlue)

    print("lower: ",lower_rangeBlue)
    print("upper: ",higher_rangeBlue)
    # print(redValue)

    # if(redValue>15000 or blueValue>15000):
    #     if(redValue>blueValue):
    #         print("RED")
    #         newIdc = maxContour(contoursRed)
    #     elif(blueValue>redValue):
    #         print ("BLUE")
    #         newIdc = maxContour(contoursBlue)

    # wc = math.sqrt(max_area)
    # wc = int(1.5 * wc)
    # print "wc: ", wc
    # M = cv2.moments(c[idc])
    # cx = int(M['m10']/M['m00'])
    # cy = int(M['m01']/M['m00'])
    # roi1 = frame[cy-(wc/2):cy+(wc/2), cx-(wc/2):cx+(wc/2)]
    # mask[cy-5:cy+5, cx-5:cx+5] = 155
    frameRed = frame
    frameBlue = frame
    cv2.drawContours(frameBlue, contoursBlue, blueIndex, (255, 0, 0), 2)
    cv2.drawContours(frameRed, contoursRed, redIndex, (0, 0, 255), 2)


    # cv2.imshow('roi', roi1)
    cv2.imshow('img', frameRed)
    cv2.imshow('img2',frameBlue)
    cv2.imshow('maskRed', maskRed)
    cv2.imshow('maskBlue', maskBlue)
    if cv2.waitKey(1)  == 27:
        break
cv2.destroyAllWindows()
cap.release()