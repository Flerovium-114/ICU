#!/usr/bin/env python

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, img = cap.read()
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(img_hsv, (0,90,50), (5,255,255))
    mask2 = cv2.inRange(img_hsv, (175,90,50), (180,255,255))

    mask = cv2.bitwise_or(mask1, mask2)

    contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        print (area)
        if area > 800:
            x,y,w,h = cv2.boundingRect(c)
            frame = cv2.rectangle(img, (x,y), (x+w,x+h), (0,0,255), 5)
            cv2.imshow("Webcam", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

