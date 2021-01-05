import sys
import cv2
import numpy as np


cap = cv2.VideoCapture(0)
while True:
    _,src = cap.read()

    height, width, _ = src.shape
    # Loads an image
    # Check if image is loaded fine

    gray = cv2.cvtColor( src, cv2.COLOR_BGR2GRAY )
    # gray = cv2.bitwise_not(gray)

    gray = cv2.bilateralFilter( gray, 1, 10, 120 )

    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    
    thresh1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 255, 2)
    retval, dst	=	cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)

    kernel = np.ones((10,1), np.uint8)
    thresh1 = cv2.dilate(thresh1, kernel, iterations=1)

    kernel = np.ones((10,1), np.uint8)
    thresh1 = cv2.erode(thresh1, kernel, iterations=1)

    kernel = np.ones((5,5), np.uint8)
    thresh1 = cv2.erode(thresh1, kernel, iterations=1)

    kernel = np.ones((20,1), np.uint8)
    thresh1 = cv2.dilate(thresh1, kernel, iterations=1)

    kernel = np.ones((25,1), np.uint8)
    thresh1 = cv2.erode(thresh1, kernel, iterations=1)

    kernel = np.ones((5,5), np.uint8)
    thresh1 = cv2.erode(thresh1, kernel, iterations=1)

    # invert the black and white image for the LineDetection
    thresh1 = cv2.bitwise_not(thresh1)

    # Increase number -> less noise
    edge  = cv2.Canny( thresh1, 100, 210 )

    kernel1 = np.ones((3,3), np.uint8)
    edges = cv2.dilate(edge, kernel1, iterations=2)

    # edges = cv2.fastNlMeansDenoising(edges)
    # edges  = cv2.Canny( edges, 10, 210 )

    kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( 7, 7 ) )
    kernel = np.ones((3,3), np.uint8)

    closed = cv2.morphologyEx( edges, cv2.MORPH_CLOSE, kernel )

    contours, h = cv2.findContours( closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 1, minLineLength=height - 100, maxLineGap=300)
    # Draw lines on the image
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if (y1 >= height - 1 and y2 == 0 and abs(x1 - x2) < 10):
            cv2.line(src, (x1, y1), (x2, y2), (255, 0, 0), 3)

    cv2.imshow("detected circles", src)
    cv2.imshow( 'edges', edges )
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
