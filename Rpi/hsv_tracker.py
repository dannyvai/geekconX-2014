import time
import thread
import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider

#tracking varaiables
x = 0
y = 0

img_width = 640
img_height = 480

def drawObject(img,x,y):
	global img_width,img_height

	cv2.circle(img,(x,y),20,(0,255,0),3)
	if(y-25>0):
		cv2.line(img,(x,y),(x,y-25),(0,255,0),2)
	else:
		cv2.line(img,(x,y),(x,0),(0,255,0),2)

	if(y+25<img_height):
		cv2.line(img,(x,y),(x,y+25),(0,255,0),2)
	else:
		cv2.line(img,(x,y),(x,img_height),(0,255,0),2)

	if(x-25>0):
		cv2.line(img,(x,y),(x-25,y),(0,255,0),2)
	else:
		cv2.line(img,(x,y),(0,y),(0,255,0),2)

	if(x+25 < img_width):
		cv2.line(img,(x,y),(x+25,y),(0,255,0),2)
	else:
		cv2.line(img,(x,y),(img_width,y),(0,255,0),2)

	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.putText(img,str(x)+','+str(y),(x,y+30),font,1,(0,255,0),2)

#Sliders variables
h_min_val,h_max_val = 0,256
s_min_val,s_max_val = 0,256
v_min_val,v_max_val= 0,256
h_min_slide = 0
h_max_slide = 0
s_min_slide = 0
s_max_slide = 0
v_min_slide = 0
v_max_slide = 0

def create_sliders():
	global h_min_slide,s_min_slide,v_min_slide,h_max_slide,s_max_slide,v_max_slide
	H_min = plt.axes([0.25, 0.1, 0.65, 0.03])
	H_max = plt.axes([0.25, 0.15, 0.65, 0.03])
	h_min_slide = Slider(H_min, 'H min', 1, 256)
	h_max_slide = Slider(H_max,'H max', 1, 256)
	h_min_slide.on_changed(update)
	h_max_slide.on_changed(update)

	V_min = plt.axes([0.25, 0.20, 0.65, 0.03])
	V_max = plt.axes([0.25, 0.25, 0.65, 0.03])
	v_min_slide = Slider(V_min, 'V min', 1, 256)
	v_max_slide = Slider(V_max, 'V max', 1, 256)
	v_min_slide .on_changed(update)
	v_max_slide .on_changed(update)

	S_min  = plt.axes([0.25, 0.30, 0.65, 0.03])
	S_max = plt.axes([0.25, 0.35, 0.65, 0.03])
	s_min_slide = Slider(S_min, 'S min', 1, 256)
	s_max_slide = Slider(S_max, 'S max', 1, 256)
	s_min_slide .on_changed(update)
	s_max_slide .on_changed(update)
	
	plt.show()


def update(val):
    global h__min_val,s_min_val,v_min_val,h_max_val,s_max_val,v_max_val
    h_min_val = int(h_min_slide.val)
    h_max_val = int(h_max_slide.val)
    s_minval = int(s_min_slide.val)
    s_maxval = int(s_max_slide.val)
    v_min_val = int(v_min_slide.val)
    v_max_val = int(v_max_slide.val)



thread.start_new_thread( create_sliders, () )
time.sleep(1)
cam = cv2.VideoCapture(1)

def morph(threshold):
	erode_ker = np.ones((3,3),np.uint8)
	dilate_ker = np.ones((8,8),np.uint8)

	erosion = cv2.erode(threshold,erode_ker,iterations = 2)
	dilation = cv2.dilate(erosion,dilate_ker,iterations = 1)
	return dilation


while True:
	s, img_rgb = cam.read()
	img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

	threshold = cv2.inRange(img_hsv,(h_min_val,s_min_val,v_min_val),(h_max_val,s_max_val,v_max_val))

	threshold = morph(threshold)
	drawObject(threshold,100,100)
	cv2.imshow("test",threshold)
	cv2.waitKey(1)