import numpy as np
import cv2
import robot_control as rbc
import time
import math

cam = cv2.VideoCapture(1)

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

loop_n = 0

def mov_to_face(x,y,w,h):
	global loop_n
	loop_n = loop_n + 1

	if loop_n < 10:
		return
        center = (320,240)
        face_center = (x+w/2,y+h/2)

        print center[0]-face_center[0],center[1]-face_center[1]
        times = 3
	was_movment = 0

        if center[0]-face_center[0] < -w/2:
		was_movment = 1
                for i in range(0,times):
                        rbc.mov_left()
        elif center[0]-face_center[0] > w/2:
		was_movment = 1
                for i in range(0,times):
                        rbc.mov_right()

        if center[1]-face_center[1] < -h/2:
                #for i in range(0,times):
		was_movment = 1
                rbc.mov_up()
        elif center[1]-face_center[1] > h/2:
                #for i in range(0,times):
		was_movment = 1
                rbc.mov_down()

	if was_movment == 0:
		if w > 170 and h > 170:
			for i in range(0,times):
				rbc.claw_close()	
		rbc.turn_on()
	else:
#		rbc.claw_open()
		rbc.turn_off()
#img = cv2.imread('sachin.jpg')
i = 0
while True:
	i = i + 1
	time.sleep(0.01)
	s,img = cam.read()
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	faces = face_cascade.detectMultiScale(gray, 1.3, 5)
	if faces is not None and len(faces) == 1:
		
		for (x,y,w,h) in faces:
		    print x,y,w,h
		    
		    mov_to_face(x,y,w,h)

		    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
		    roi_gray = gray[y:y+h, x:x+w]
		    roi_color = img[y:y+h, x:x+w]
		    eyes = eye_cascade.detectMultiScale(roi_gray)	
		    for (ex,ey,ew,eh) in eyes:
		        cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
	else:
		if i%5 == 0:
			print "claw_open"
			rbc.claw_open()
			rbc.turn_off()
	cv2.imshow('img',img)
	cv2.waitKey(1)

cv2.destroyAllWindows()


