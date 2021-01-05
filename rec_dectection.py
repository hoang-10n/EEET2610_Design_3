import cv2
import numpy as np

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        _,src = cap.read()
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        ## Split the H channel in HSV, and get the red range
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        h[h<150]=0
        h[h>180]=0

        # normalize, do the open-morp-op
        normed = cv2.normalize(h, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3,3))
        opened = cv2.morphologyEx(normed, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(opened, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            ## Get the stright bounding rect
            bbox = cv2.boundingRect(cnt)
            x,y,w,h = bbox
            if w < 100 or h < 100 or w*h < 10000:
                continue

            ## Draw rect
            cv2.rectangle(src, (x,y), (x+w,y+h), (255,0,0), 1, 16)

        #     ## Get the rotated rect
        #     rbox = cv2.minAreaRect(cnt)
        #     (cx,cy), (w,h), rot_angle = rbox

        cv2.imshow("detected rectangle", src)

        if cv2.waitKey(1) == ord("q"):
            break