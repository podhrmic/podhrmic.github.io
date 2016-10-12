import socket
import re
import time

class VehicleSocket:
	# Open socket to vehicle
	def connect(self):
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind(('', 25565))
		s.listen(1)

		(self.c, addr) = s.accept()
		data = self.c.recv(64) # on connect vehicle sends data
		if data == 'hi': # on first connect vehicle says hi
			print("Vehicle ready to be initiated")
			return "ready"

	# calls initialization procedure
	def init(self): # slow start init cycle for vehicle
		self.c.sendall('init')
		data = self.c.recv(64) # vehicle responds with init,done when init'd
		if data == 'init,done':
			print("Vehicle initiated")
			return "ready"

	# calls stopping routine
	def stop(self):
		self.c.sendall('stop')

	# Send parameters to vehicle
	def sendParams(self, l_Delta, r_Delta, l_Omega, r_Omega): # send params for vehicle to flap to
		self.c.sendall("params")
		data = self.c.recv(64)
		tmp = "{:.2f},{:.2f};{:.2f},{:.2f};\n".format(l_Delta, r_Delta, l_Omega, r_Omega) # format is 'lDelta, rDelta; lOmega, rOmega;\n'
		self.c.sendall(tmp)
		time.sleep(1)
		data = self.c.recv(64) # vehicle responds with confirmation of flaps.
		time.sleep(1)
		data = data.strip()
		spl = re.split(r'[,;]+', data)
		del spl[-1]
		data = [float(i) for i in spl]
		if (data == [l_Delta, r_Delta, l_Omega, r_Omega]):
			return('sent')			
		else:
			print("Not sent successfully")
			return('fail')

	# Close sockets
	def close(self):
		self.c.sendall('q')
		self.c.close()
