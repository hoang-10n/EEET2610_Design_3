import cv2
import numpy as np

cap = cv2.VideoCapture(0)
_, frame = cap.read()
height, width, depth = frame.shape
r,g,b = frame[(int)(height/2), (int)(width/2)]
while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([0,0,0])
    upper_blue = np.array([r,g,b])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame,frame, mask = mask)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    if cv2.waitKey(27) & 0xFF == ord('q') :
        break