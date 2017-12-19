import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('camera',gray)
    k = cv2.waitKey(1) & 0xFF
    if k == 13:
        break

r = cv2.selectROI(gray)

template = gray[r[1]:r[1]+r[3],r[0]:r[0]+r[2]]
w,h = template.shape[::-1]

cv2.imshow('template', template)

while(1):
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.8
    loc = np.where(res >= threshold)

    for pt in zip(*loc[::-1]):
        cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (255,0,0), 2)

    cv2.imshow('Found',frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
