import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider
import thread


h_val = 0
s_val = 0
v_val = 0

h_slide = 0
s_slide = 0
v_slide = 0

x = 0
y = 0

def create_sliders():
	global h_slide,s_slide,v_slide
	H = plt.axes([0.25, 0.1, 0.65, 0.03])
	S  = plt.axes([0.25, 0.15, 0.65, 0.03])
	V = plt.axes([0.25, 0.20, 0.65, 0.03])
	
	h_slide = Slider(H, 'H', 0.4, 255)
	s_slide = Slider(S, 'S', 0.4, 255)
	v_slide = Slider(V, 'V',0.1,255)

	h_slide.on_changed(update)
	s_slide .on_changed(update)
	v_slide .on_changed(update)
	plt.show()


def update(val):
    global h_val,s_val,v_val
    h_val = int(h_slide.val)
    s_val = int(s_slide.val)
    v_val = int(v_slide.val)

thread.start_new_thread( create_sliders, () )

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

	threshold = cv2.inRange(img_hsv,(h_val,s_val,v_val),(255,255,255))

	threshold = morph(threshold)

	cv2.imshow("test",threshold)
	cv2.waitKey(1)