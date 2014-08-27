import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

h_val = 0
s_val = 0
v_val = 0

H = plt.axes([0.25, 0.1, 0.65, 0.03])
S  = plt.axes([0.25, 0.15, 0.65, 0.03])
V = plt.axes([0.25, 0.20, 0.65, 0.03])

h_slide = Slider(H, 'H', 0.4, 255)
s_slide = Slider(S, 'S', 0.4, 255)
v_slide = Slider(V, 'V',0.1,255)

def update(val):
    h_val = int(h_slide.val)
    s_val = int(s_slide.val)
    v_val = int(v_slide.val)


h_slide.on_changed(update)
s_slide .on_changed(update)
v_slide .on_changed(update)
plt.show()
