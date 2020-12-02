import cv2
import numpy as np
from copy import deepcopy
import math

cap = cv2.VideoCapture(0)
while True:
    _, imgobj = cap.read()

    gray = cv2.cvtColor(imgobj, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow("image")
    cv2.imshow("image", gray)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    cv2.imshow("image", blurred)
    canny = cv2.Canny(blurred, 20, 40)
    cv2.imshow("image", canny)
    kernel = np.ones((3,3), np.uint8)
    dilated = cv2.dilate(canny, kernel, iterations=2)
    cv2.imshow("image", blurred)

    (contours, hierarchy) = cv2.findContours(dilated.copy(),
                                             cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)


    candidates = []
    hierarchy = hierarchy[0]

    index = 0
    pre_cX = 0
    pre_cY = 0
    center = []
    for component in zip(contours, hierarchy):
        contour = component[0]
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.1 * peri, True)
        area = cv2.contourArea(contour)
        corners = len(approx)

        # compute the center of the contour
        M = cv2.moments(contour)

        if M["m00"]:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX = None
            cY = None

        if 14000 < area < 20000 and cX is not None:
            tmp = {'index': index, 'cx': cX, 'cy': cY, 'contour': contour}
            center.append(tmp)
            index += 1

    center.sort(key=lambda k: (k.get('cy', 0)))
    row1 = center[0:3]
    row1.sort(key=lambda k: (k.get('cx', 0)))
    row2 = center[3:6]
    row2.sort(key=lambda k: (k.get('cx', 0)))
    row3 = center[6:9]
    row3.sort(key=lambda k: (k.get('cx', 0)))

    center.clear()
    center = row1 + row2 + row3

    for component in center:
        candidates.append(component.get('contour'))

    cv2.drawContours(imgobj, candidates, -1, (0, 0, 255), 3)
    cv2.imshow("image", imgobj)
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
