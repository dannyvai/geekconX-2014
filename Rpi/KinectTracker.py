#!/usr/bin/env python
import freenect
import cv
import frame_convert
import numpy as np
import cv2

threshold = 44
current_depth = 861

################
#	ImageProcesssing #
################
img_width = 640
img_height = 480

def drawObject(img,x,y):
	global img_width,img_height
	print img
	print x
	print y
	cv2.circle(img,(x,y),20,(0,0,255),3)
	if(y-25>0):
		cv2.line(img,(x,y),(x,y-25),(0,0,255),2)
	else:
		cv2.line(img,(x,y),(x,0),(0,0,255),2)

	if(y+25<img_height):
		cv2.line(img,(x,y),(x,y+25),(0,0,255),2)
	else:
		cv2.line(img,(x,y),(x,img_height),(0,0,255),2)

	if(x-25>0):
		cv2.line(img,(x,y),(x-25,y),(0,0,255),2)
	else:
		cv2.line(img,(x,y),(0,y),(0,0,255),2)

	if(x+25 < img_width):
		cv2.line(img,(x,y),(x+25,y),(0,0,255),2)
	else:
		cv2.line(img,(x,y),(img_width,y),(0,0,255),2)

	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.putText(img,str(x)+','+str(y),(x,y+30),font,1,(0,0,255),4)


def morph(threshold):
	erode_ker = np.ones((3,3),np.uint8)
	dilate_ker = np.ones((8,8),np.uint8)

	erosion = cv2.erode(threshold,erode_ker,iterations = 1)
	dilation = cv2.dilate(erosion,dilate_ker,iterations = 1)
	return dilation

def trackFilteredObject(threshold,img):
	temp = threshold[:,:].copy()
	refArea = 0
	objectFound = False
	contours,hierarchy = cv2.findContours(temp,cv2.cv.CV_RETR_CCOMP,cv2.cv.CV_CHAIN_APPROX_SIMPLE)
	if len(contours) > 0 and len(contours) < 100:
		try:
			for idx in range(0,len(hierarchy[0])):
				m = cv2.moments(contours[idx])
				x,y,w,h = cv2.boundingRect(contours[idx])
				detect_area = w*h # cv2.contourArea(contours[idx])
				#cv2.drawContours(img,contours[idx],-1,(0,0,255),5)
				#print detect_area
				if detect_area > 300:                                      
					print idx, x,y,w,h
					#cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
					cv2.rectangle(threshold,(x,y),(x+w+5,y+h+5),(0,0,255),2)
					print detect_area 
					
					objectFound = True
					refArea = detect_area
					track_x = x
					track_y = y
					#print track_x,track_y
					drawObject(img,track_x,track_y)
		except:
			#import pdb;pdb.set_trace()
			pass

	else:
		objectFound = False

	#if objectFound is True:
	#	drawObject(img,track_x,track_y)





def change_threshold(value):
    global threshold
    threshold = value


def change_depth(value):
    global current_depth
    current_depth = value


def show_depth(img_rgb):
    global threshold
    global current_depth

    depth, timestamp = freenect.sync_get_depth()
    depth = 255 * np.logical_and(depth >= current_depth - threshold,
                                 depth <= current_depth + threshold)
    depth = depth.astype(np.uint8)

    depth = morph(depth)
    trackFilteredObject(depth,img_rgb)

    image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]),cv.IPL_DEPTH_8U,1)
    cv.SetData(image, depth.tostring(), depth.dtype.itemsize * depth.shape[1])
    cv.ShowImage('Depth', image)


def show_video():
    cv.ShowImage('Video', frame_convert.video_cv(freenect.sync_get_video()[0]))

def get_video():
    if freenect.sync_get_video() is not None:
    	return frame_convert.video_cv(freenect.sync_get_video()[0])
    return None

cv.NamedWindow('Depth')
cv.NamedWindow('Video')
cv.CreateTrackbar('threshold', 'Depth', threshold,     500,  change_threshold)
cv.CreateTrackbar('depth',     'Depth', current_depth, 2048, change_depth)

print('Press ESC in window to stop')


while 1:
    
    img_rgb = get_video()
    if img_rgb is not None:
    	img_rgb = np.asarray(img_rgb)
    show_depth(img_rgb)
    show_video()
    if cv.WaitKey(10) == 27:
        break
