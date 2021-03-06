import cv2
import numpy as np
import time
cam = cv2.VideoCapture(1)

ts = time.time()
minr = 50
maxr = 1000

while True:
	s,img = cam.read()
#img = cv2.imread('opencv_logo.png',0)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img = cv2.medianBlur(img,3)	
	cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
	#cimg = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
	toc = time.time() - ts
	#if toc > 2:
	#	minr = 100 # 10*(toc/2)
	#	maxr = 200# + int(50*(toc/2))
	#	print minr,maxr
	try:
		circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1,30,param1=200,param2=50,minRadius=minr,maxRadius=maxr)

		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
    			# draw the outer circle
    			cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    			# draw the center of the circle
    			cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

	except :
		pass

	cv2.imshow('detected circles',cimg)
	cv2.waitKey(1)

cv2.destroyAllWindows()
