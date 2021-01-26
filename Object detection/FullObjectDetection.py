# -*- coding: utf-8 -*-
import sys
import cv2
import time
import numpy as np
import paho.mqtt.publish as publish

BROKER_LOCATION = "192.168.4.1"

DELAY = 5

"""
sudo apt-get install python-opencv
sudo apt-get install python-matplotlib
"""


def findCube(rgb):
    MORPH = 7
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

    gray = cv2.bilateralFilter(gray, 1, 10, 120)

    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edge = cv2.Canny(gray, 60, 100)

    kernel1 = np.ones((3, 3), np.uint8)

    edges = cv2.dilate(edge, kernel1, iterations=1)
    edges = cv2.dilate(edges, kernel1, iterations=1)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH, MORPH))

    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    contours, hieriarchy = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cont in contours:
        bbox = cv2.boundingRect(cont)

        x, y, w, h = bbox

        area = cv2.contourArea(cont)
        if area > 1000:
            arc_len = cv2.arcLength(cont, True)

            approx = cv2.approxPolyDP(cont, 0.01 * arc_len, True)

            rbox = cv2.minAreaRect(cont)
            (cx, cy), (cw, ch), rot_angle = rbox
            if area - 1500 < cw * ch < area + 1500 and len(approx) == 4:
                cv2.rectangle(rgb, (x, y), (x + w, y + h), (255, 0, 0), 1, 16)
                cv2.drawContours(rgb, [approx], -1, (25, 0, 0), -1)
                # print(approx)

            if (len(approx) in range(4, 8)):
                cv2.drawContours(rgb, [approx], -1, (255, 0, 0), -1)
                cv2.putText(rgb, "Cube", ((int)(cx) - 20, (int)(cy) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
                print("Cube !!!")
                publish.single("Design3/Object", "cube", hostname = BROKER_LOCATION)
                break
            else:
                publish.single("Design3/Object", "cylinder", hostname = BROKER_LOCATION)
                print("Not cube !!")
                print("Cylinder !!")
                break

    cv2.imshow('edges', edges)
    cv2.imshow('rgb', rgb)
    if cv2.waitKey(99) & 0xFF == ord('c'):
        pass


def findCircle(rgb):
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

    gray = cv2.medianBlur(gray, 5)

    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=150, param2=30,
                               minRadius=50, maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(rgb, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(rgb, center, radius, (255, 0, 255), -1)
            cv2.putText(rgb, "Sphere", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 2)
        publish.single("Design3/Object", "sphere", hostname = BROKER_LOCATION)
        print("Sphere !!!")
        cv2.imshow("detected circles", rgb)
        if cv2.waitKey(99) & 0xFF == ord('c'):
            pass

    else:
        print("Not sphere !!!")
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        ## mask of green (36,25,25) ~ (86, 255,255)
        # mask = cv2.inRange(hsv, (36, 25, 25), (86, 255,255))
        mask = cv2.inRange(hsv, (70,40,40), (125,255,255))

        ## slice the green
        imask = mask>0
        rg = np.zeros_like(rgb, np.uint8)
        rg[imask] = rgb[imask]
        findCube(rg)

def detect():
    # img = cv2.imread("Cube.jpg", 1)
    cap = cv2.VideoCapture(0)
    # while True:
    _, img = cap.read()
    findCircle(img)
    time.sleep(0.5)

detect()