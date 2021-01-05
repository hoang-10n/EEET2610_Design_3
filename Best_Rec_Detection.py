#-*- coding: utf-8 -*-

import cv2
import time
import numpy as np

"""
sudo apt-get install python-opencv
sudo apt-get install python-matplotlib
"""

##################
DELAY = 0.02
USE_CAM = 1
IS_FOUND = 0

MORPH = 7
CANNY = 250
##################
_width  = 700.0
_height = 500.0
_margin = 0.0
##################

if USE_CAM: video_capture = cv2.VideoCapture(0)

corners = np.array(
	[
		[[  		_margin, _margin 			]],
		[[ 			_margin, _height + _margin  ]],
		[[ _width + _margin, _height + _margin  ]],
		[[ _width + _margin, _margin 			]],
	]
)

pts_dst = np.array( corners, np.float32 )

while True :

	# if USE_CAM :
	# 	ret, rgb = video_capture.read()
	# else :
	ret = 1
	rgb = cv2.imread( "Untitled.jpg", 1 )

	if ( ret ):

		gray = cv2.cvtColor( rgb, cv2.COLOR_BGR2GRAY )
		# gray = cv2.bitwise_not(gray)

		gray = cv2.bilateralFilter( gray, 1, 10, 120 )

		gray = cv2.GaussianBlur(gray, (5, 5), 0)

		# thresh1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 255, 2)
		# retval, dst	=	cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)

		# kernel = np.ones((10,1), np.uint8)
		# thresh1 = cv2.dilate(thresh1, kernel, iterations=1)

		# kernel = np.ones((10,1), np.uint8)
		# thresh1 = cv2.erode(thresh1, kernel, iterations=1)

		# kernel = np.ones((5,5), np.uint8)
		# thresh1 = cv2.erode(thresh1, kernel, iterations=1)

		# kernel = np.ones((20,1), np.uint8)
		# thresh1 = cv2.dilate(thresh1, kernel, iterations=1)

		# kernel = np.ones((25,1), np.uint8)
		# thresh1 = cv2.erode(thresh1, kernel, iterations=1)

		# kernel = np.ones((5,5), np.uint8)
		# thresh1 = cv2.erode(thresh1, kernel, iterations=1)

		# # invert the black and white image for the LineDetection
		# thresh1 = cv2.bitwise_not(thresh1)

		# Increase number -> less noise
		edge  = cv2.Canny( gray, 30, 100 )

		kernel1 = np.ones((3,3), np.uint8)

		edges = cv2.dilate(edge, kernel1, iterations=1)
		edges = cv2.dilate(edges, kernel1, iterations=1)

		# edges = cv2.fastNlMeansDenoising(edges)
		# edges  = cv2.Canny( edges, 10, CANNY )

		kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( MORPH, MORPH ) )
		# kernel = np.ones((3,3), np.uint8)                                                                                    02

		closed = cv2.morphologyEx( edges, cv2.MORPH_CLOSE, kernel )

		contours, hieriarchy = cv2.findContours( closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE )

		for cont in contours:
			bbox = cv2.boundingRect(cont)
			
			x,y,w,h = bbox

			area = cv2.contourArea( cont )
			if area > 1000 :
				arc_len = cv2.arcLength( cont, True )

				approx = cv2.approxPolyDP( cont, 0.02 * arc_len, True )

			# if ( len( approx ) == 4 ):
				# M = cv2.moments( cont )
				# cX = int(M["m10"] / M["m00"])
				# cY = int(M["m01"] / M["m00"])
				# cv2.putText(rgb, "Center", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

				# pts_src = np.array( approx, np.float32 )

				# hie, status = cv2.findHomography( pts_src, pts_dst )
				# out = cv2.warpPerspective( rgb, h, ( int( _width + _margin * 2 ), int( _height + _margin * 2 ) ) )


				rbox = cv2.minAreaRect(cont)
				(cx,cy), (cw,ch), rot_angle = rbox
				if (cw * ch > area - 1500 and cw * ch < area + 1500 and len(approx) == 4): # 1000 is error number
					cv2.rectangle(rgb, (x,y), (x+w,y+h), (255,0,0), 1, 16)
					cv2.drawContours( rgb, [approx], -1, ( 255, 0, 0 ), -1 )
					# print(approx)
					i = 0
					for lst in approx:
						cv2.circle(rgb, (lst[0][0], lst[0][1]), 3, (0,255,0), -1)
						cv2.putText(rgb, str(i), (lst[0][0], lst[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
						i = i + 1
						# cv2.putText(rgb, "Rectangle", ((int)(cx) - 20, (int)(cy) - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

				if (len ( approx ) == 6):
					cv2.drawContours( rgb, [approx], -1, ( 255, 0, 0 ), -1 )
					cv2.putText(rgb, "Cube", ((int)(cx) - 20, (int)(cy) - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			# lines = cv2.HoughLinesP(edges, 1, np.pi/180, 1, minLineLength=50, maxLineGap=200)
			# # Draw lines on the image
			# for line in lines:
			# 	x1, y1, x2, y2 = line[0]
			# 	cv2.line(rgb, (x1, y1), (x2, y2), (255, 0, 0), 3)
			# for i in range(0, len(lines) - 1):
			# 	for j in range(i, len(lines)):
			# 		xi1, yi1, xi2, yi2 = lines[i][0]
			# 		xj1, yj1, xj2, yj2 = lines[j][0]
			# 		if ((xj1 - xj2) == 0 or (yj1 - yj2) == 0):
			# 			continue
			# 		if ((xi1 - xi2) == (xj1 - xj2) and (yi1 - yi2) == (yj1 - yj2)):
			# 			cv2.line(rgb, (xi1, yi1), (xi2, yi2), (255, 0, 0), 3)
			# 			cv2.line(rgb, (xj1, yj1), (xj2, yj2), (255, 0, 0), 3)

			# else : pass

		corners = cv2.goodFeaturesToTrack(gray, 50, 0.01, 50)

		for corner in corners:
			x, y = corner.ravel()
			cv2.circle(rgb, (x, y), 3, (0,0,255), -1)

		#cv2.imshow( 'closed', closed )
		#cv2.imshow( 'gray', gray )
		# cv2.namedWindow( 'edges', cv2.CV_WINDOW_AUTOSIZE )
		cv2.imshow( 'edges', edges )
		# cv2.imshow( 'thresh1', dst )

		# cv2.namedWindow( 'rgb', cv2.CV_WINDOW_AUTOSIZE )
		cv2.imshow( 'rgb', rgb )

		# if IS_FOUND :
		# 	cv2.namedWindow( 'out', cv2.CV_WINDOW_AUTOSIZE )
		# 	cv2.imshow( 'out', out )

		if cv2.waitKey(27) & 0xFF == ord('q') :
			break

		if cv2.waitKey(99) & 0xFF == ord('c') :
			current = str( time.time() )
			cv2.imwrite( 'ocvi_' + current + '_edges.jpg', edges )
			cv2.imwrite( 'ocvi_' + current + '_gray.jpg', gray )
			cv2.imwrite( 'ocvi_' + current + '_org.jpg', rgb )

		time.sleep( DELAY )

	else :
		break

if USE_CAM : video_capture.release()
cv2.destroyAllWindows()