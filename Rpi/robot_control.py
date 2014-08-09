import serial
from threading import Thread
import time

ser = serial.Serial('/dev/ttyACM0',baudrate=9600)

tts = 0.01


#t,g up down
#q,a left right

def mov_left():
        t = Thread(target=mov_left_t)
        t.start()

def mov_right():
        t = Thread(target=mov_right_t)
        t.start()

def mov_up():
	t = Thread(target=mov_up_t)
        t.start()
def mov_down():
	t = Thread(target=mov_down_t)
        t.start()


#def send_down_thread():
#        ts = time.time()
#        while time.time() - ts < tts:
#                ser.write(back)



def mov_left():
	#ts = time.time()
	#while time.time() - ts < tts:
	ser.write('q')

def mov_right():
	#ts = time.time()
        #while time.time() - ts < tts:
        ser.write('a')

def mov_up_t():
#	ts = time.time()
#	while time.time() - ts < tts:
	ser.write('t')

def mov_down_t():
#        ts = time.time()
#        while time.time() - ts < tts:
         ser.write('g')

def turn_on():
	ser.write('u')

def turn_off():
	ser.write('j')

def claw_open():
	ser.write('h')

def claw_close():
	ser.write('y')

def mov_z():
	pass

