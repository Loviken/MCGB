# Interface to Nao

import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import qi

# Constants
DOF 		= 26
STIFFNESS 	= 1.
SPEED	 	= .4
MOVE_TIME 	= 1.

RANGES = np.array( [[-1., 1.], #[-2.0857, 2.0857],		#0	HeadYaw
					[-0.6720, 0.5149],		#1	HeadPitch
					[-2.0857, 2.0857],		#2	LShoulderPitch
					[-0.3142, 1.3265],		#3	LShoulderRoll
					[-2.0857, 2.0857],		#4	LElbowYaw
					[-1.5446,-0.0349],		#5	LElbowRoll
					[-1.8238, 1.8238],		#6	LWristYaw
					[-1, 1],				#7	LHand
					[-1.145303, 0.740810],	#8	LHipYawPitch
					[-0.379472, 0.790477],	#9	LHipRoll
					[-1.535889, 0.484090],	#10 LHipPitch
					[-0.092346, 2.112528],	#11 LKneePitch  (?)
					[-1.189516, 0.922747],	#12 LAnklePitch (?)
					[-0.39, 	0.68],		#13 LAnkleRoll
					[-1.145303, 0.740810],	#14	RHipYawPitch (Same servo as 8)
					[-0.790477, 0.379472],	#15 RHipRoll
					[-1.535889, 0.484090],  #16 RHipPitch
					[-0.103083, 2.120198],	#17 RKneePitch
					[-1.186448, 0.932056],	#18 RAnklePitch
					[-0.68, 	0.39],		#19 RAnkleRoll
					[-2.0857, 2.0857],  	#20 RShoulderPitch
					[-1.3265, 0.3142],		#21 RShoulderRoll
					[-2.0857, 2.0857],		#22 RElbowYaw
					[ 0.0349, 1.5446],		#23 RElbowRoll
					[-1.8238, 1.8238],		#24 RWristYaw
					[-1, 1]])				#25 RHand

# From: http://doc.aldebaran.com/1-14/family/robots/joints_robot.html#robot-joints-v4-head-joints


class Body:
	def __init__(self, proxy):
		
		self.session = proxy
		self.motion_service = proxy.service("ALMotion")
		self.memory_service = proxy.service("ALMemory")

		# Deactivate fall reflex
		self.motion_service.setFallManagerEnabled(False)
		self.angle_goal = np.array([0.]*DOF)
		
		
	# Transform q to angles, and set as goal
	def set_goal(self, q):
		goal_angles 	= (RANGES[:,1] - RANGES[:,0])*q + RANGES[:,0]
		self.angle_goal	= goal_angles
		
		
	# Get current posture q
	def get_q(self):
		angles 	= np.array(self.motion_service.getAngles('Body', True))
		q 		= (angles - RANGES[:,0])/(RANGES[:,1] - RANGES[:,0])
		return q
		

	def step(self, stiff = STIFFNESS):
		t_start = time.time()
		
		self.motion_service.setStiffnesses("Body", stiff)
		self.motion_service.setAngles("Body", self.angle_goal.tolist(), SPEED)
		
		time.sleep(MOVE_TIME)
		
		# Stop all motions
		self.set_goal(self.get_q())	# current posture is goal
		self.motion_service.setAngles("Body", self.angle_goal.tolist(), SPEED)
		
		# Collect inclination when converged
		x = self.get_x()
		
		while True:
			x_old 	= x
			
			time.sleep(0.1)
			x = self.get_x()
			
			if np.linalg.norm(x-x_old) < 0.02:
				break
		
		return x, self.get_q()
		
	def relax(self):
		self.motion_service.setStiffnesses("Body", 0.)
		
		
	# PRIVATE #
	# Returns x in raw form
	def get_x(self):
		memory_service = self.memory_service

		# Get the Accelerometers Values
		AccX = memory_service.getData("Device/SubDeviceList/InertialSensor/AccX/Sensor/Value")
		AccY = memory_service.getData("Device/SubDeviceList/InertialSensor/AccY/Sensor/Value")
		AccZ = memory_service.getData("Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value")

		x = np.array([AccX, AccY, -AccZ])
		
		return x/np.linalg.norm(x)


#################################################
# Small test
if False:
	IP_ADRESS 	= "192.168.43.184"
	PORT		= 9559
	v5			= False
	
	if v5:
		motionProxy = ALProxy("ALMotion", IP_ADRESS, PORT)
		memoryProxy = ALProxy("ALMemory", IP_ADRESS, PORT)
		
		prox = [motionProxy,memoryProxy]
		
	else:
		prox = qi.Session()
		prox.connect("tcp://" + IP_ADRESS + ":" + str(PORT))
		
		
	b = Body(prox, v5)
	b.relax()
