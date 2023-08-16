#***Before using this example the motor/controller combination must be
#***tuned and the settings saved to the Roboclaw using IonMotion.
#***The Min and Max Positions must be at least 0 and 50000

import time
from roboclaw_3 import Roboclaw

#Windows comport name
#rc = Roboclaw("COM3",115200)
rc = Roboclaw("/dev/RoboclawCOM",115200)
#Linux comport name


def displayspeed():
	enc1 = rc.ReadEncM1(address)
	enc2 = rc.ReadEncM2(address)
	speed1 = rc.ReadSpeedM1(address)
	speed2 = rc.ReadSpeedM2(address)

	print("Speed1:"),
	print(speed1[1])

rc.Open()
address = 0x80


while(1):
	rc.SpeedM1(address,2000)
	rc.SpeedM2(address,0)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)

  
