import signal
import sys
from veh_comm import VehicleSocket

# vehicle communication object
veh = VehicleSocket()

# closes socket and tells vehicle to stop
# closes python script
def close():
	veh.close()
	print("closing")
	sys.exit(0)

# handles SIGINT call by closing everything
def signal_handler(signal, frame):
	close()
	print("closing")
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Command that handles setting flap params for vehicle.
# Arguments are lD = Left Delta
# 				rD = Right Delta
# 				lO = Left Omega
# 				rO = Right Omega
# This routinte will repeat 5 times on a failure then return a 'fail'
def setFlapParams(lD, rD, lO, rO):
	r = veh.sendParams(lD, rD, lO, rO)
	if(r == 'fail'):
		for i in range(1,6):
			time.sleep(1)
			r = veh.sendParams(lD, rD, lO, rO)
			if(r == 'sent'):
				return r
	return r

# THIS ROUTINE MUST BE CALLED WHEN:
# THE VEHICLE HAS BEEN RESET
# THE VEHICLE HAS BEEN TURNED ON
# THE VEHICLE HAS BEEN STOPPED
def init():
	veh.init()
	r = setFlapParams(6.28, 6.28, 12.56, 12.56)

# This simply sets the Omega to zero. This will *stop* the vehicle
# It is generally a good idea to reset the vehicle and run init 
# when this happens.
def zero():
	r = setFlapParams(-1, -1, 1, 1)
	r = setFlapParams(-1, -1, 0, 0)

# Calls a vehicle stop routine. 
def stop():
	veh.stop()

def rotateClockwise():
	r = setFlapParams(10, 20, 40, 40)

def rotateCClockwise():
	r = setFlapParams(20, 10, 40, 40)

def attemptForward():
	r = setFlapParams(10, 10, 40, 40)

def attemptNothing():
	r = setFlapParams(20, 20, 40, 40)

# Custom values setting.
def userParams():
	print("ENTER LEFT OMEGA")
	left_omega = float(raw_input())
	print("ENTER RIGHT OMEGA")
	right_omega = float(raw_input())
	print("ENTER LEFT DELTA")
	left_delta = float(raw_input())
	print("ENTER RIGHT DELTA")
	right_delta = float(raw_input())

	setFlapParams(left_delta, right_delta, left_omega, right_omega)

#main()
veh.connect()
print("INITIALIZING VEHICLE")
init()
a = None
while(a == None):
	print("THE FOLLOWING ARE SIMPLE CONTROLLER COMMANDS:")
	print("TO ROTATE CLOCKWISE, ENTER: \"rclock\"")
	print("TO ROTATE COUNTERCLOCKWISE, ENTER: \"rcclock\"")
	print("TO ATTEMPT TO MOVE FORWARD, ENTER: \"move\"")
	print("TO STOP AND CLOSE ENTIRE PROGRAM, ENTER: \"quit\"")
	print("TO ENTER CUSTOM VALUES, ENTER: \"custom\"")
	a = raw_input()
	a = a.strip()
	if(a == "rclock"):
		rotateClockwise()
	elif(a == "rcclock"):
		rotateCClockwise()
	elif(a == "move"):
		attemptForward()
	elif(a == "quit"):
		close()
	elif(a == "custom"):
		userParams()
	else:
		attemptNothing()
	a = None