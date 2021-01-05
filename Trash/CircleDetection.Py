import sys
import cv2 as cv
import numpy as np


cap = cv.VideoCapture(0)
while True:
    _,src = cap.read()
    # Loads an image
    # Check if image is loaded fine

    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

    gray = cv.medianBlur(gray, 5)

    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                              param1=150, param2=30,
                              minRadius=1, maxRadius=40)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(src, center, radius, (255, 0, 255), 3)

    cv.imshow("detected circles", src)
    if cv.waitKey(1) == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
