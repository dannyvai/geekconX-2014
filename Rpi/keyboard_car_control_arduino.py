import Tkinter as tk
import serial
import time
import struct
import car_control_nrf_protocol as ccnp

#ser = serial.Serial('/dev/ttyACM0')

#back = struct.pack("BBBBBB",255,254,1,254,1,0)
#front = struct.pack("BBBBBB",255,254,0,254,0,0)
#timeval = 0.2

def onKeyPress(event):
    text.insert('end', 'You pressed %s\n' % (event.char, ))
    ts = time.time();
    if event.char == 's' :
#	while time.time()-ts < timeval :
#		ser.write(back)
	ccnp.send_down()
    if event.char == 'w' :
#	while time.time()-ts < timeval :
#   		ser.write(front)	
	ccnp.send_up()
root = tk.Tk()
root.geometry('300x200')
text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 12))
text.pack()
root.bind('<KeyPress>', onKeyPress)
root.mainloop()
