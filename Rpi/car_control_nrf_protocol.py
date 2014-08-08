import serial
import time
import struct
from threading import Thread

ser = serial.Serial('/dev/ttyACM1')

back = struct.pack("BBBBBB",255,254,1,254,1,1)
front = struct.pack("BBBBBB",255,254,0,254,0,1)

tts = 0.2

def send_up():
	t = Thread(target=send_up_thread)
	t.start()

def send_down():
	t = Thread(target=send_down_thread)
	t.start()

def send_up_thread():
	ts = time.time()
	while time.time() - ts < tts:
		ser.write(front)

def send_down_thread():
        ts = time.time()
        while time.time() - ts < tts:
	        ser.write(back)

