

nCars = 1
minVel4az = 1

class CarData:

	def __init__(self,x,y,x_speed,y_speed,time):
		self.x = x
		self.y = y
		self.speed_x = speed_x
		self.speed_y = speed_y
		self.det_time = time
		

for i in range(0,nCars):
	CarsData[i] = CarData(0,0,0,0,-1)

def CarTracker(CurrTime,DetectionList,DetectionTime,nDetections):
	for i in range(0,nCars):
		dT = DetectionTime - CarsData[i].det_time
		

