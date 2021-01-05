import cv2
import numpy as np
import imutils



cap = cv2.VideoCapture(0)
while True:
    _, image = cap.read()

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = 255 - gray
    ret, thresh = cv2.threshold(gray, 225, 255, cv2.THRESH_BINARY_INV)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) >= 150:        # Change 150 to 300 when demo (i will explain this at school)
        image = cv2.drawContours(image, contours, -1, (0, 0, 255), 3)
        print("black!!!")
    cv2.imshow('frame', image)
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
