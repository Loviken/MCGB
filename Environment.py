'''
Agent.Arm({graphics, random_start, armLength, dof, id_frame, d_start, wall_setting, nSteps})

Variables
	graphics		-	Use a Graphics module to show state of the arm
	random_start	- 	Find a random start posture (True/<False>)
	armLength		-	How long should the arm be? Room is 1x1.
	dof				-	Number of joints
	id_frame		- 	What Graphics sub-frame should the agent paint itself in?
	q_start			-	Start vector for arm where q = [q1 q2 ... qDoF], qi - [0,1]
	wall_setting	-	What predefined room is the agent in?
	nSteps			- 	How many steps should a motion take? (= trajectory resolution)
	
Functions
	PUBLIC
	Agent.set_goal(q)
	- The agent will try to move to 'q' in 'nSteps' steps.
		
	Agent.get_state() = [x,q]
	- Return current configuration [x,q]
		
	Agent.get_random_posture() = q
	- Get a random q that is legal. Observe that this might take some time.
	
	Agent.step()
	- Let controller using current goal until motion stops.
	return:
		failed - Was the transition successful?
	
	PRIVATE
	Agent.update()
	- One step into the future
	return: 
		observation - the current state [x,m]
		stopped		- True/False flag whether movement was aborted
		done		- True/False if movement to goal is done.
	
	Agent.set_posture(m)
	return: True/False
	- Set agent in motorstate 'q'. 
	- True/False indicate if the posture is legal.
		
	Agent.render(draw_all = True, background = None)
	- Draw current state of arm in graphics.
	- If 'draw_all' redraw walls too
	- If there is a background, redraw it too.
	
'''

import copy
import numpy as np
import pygame as pg
from pygame.locals import *
import sys
import Graphics
import BodyNao
import Rooms4
import time
import qi

#CONSTANTS
WHITE	= (255,255,255)
GRAY	= (225,225,225)
BLACK	= (0,0,0)
RED     = (255,50,50)
BLUE    = (50,50,255)
GREEN   = (50,255,50)

LINE_WIDTH = 3

class Parent(object):
	
	def __init__(self, graphics = Graphics.Basic(), id_frame = 0):
		# STATE
		self.q 		= None
		self.x 		= None
		self.q_goal = None
		self.x_goal = None
		self.x_Glob = None
		
		self.walls  = []
		
		# GRAPHICS
		#  To use graphics you need a module 'Graphics.py', to not use graphics, use graphics = 0
		self.initiateGraphics(graphics, id_frame)
		

	def initiateGraphics(self, graphics = Graphics.Basic(), id_frame = 0):
		self.graphics  = graphics						# Use this canvas for plotting etc
		if graphics != 0:
			self.id_frame = id_frame
			self.displace = np.array(graphics.get_canvas_displacement(id_frame))	# This is where the plotting starts
			self.wall_displace = np.append(self.displace , np.array([0,0]))
			self.screen    = graphics.screen
			self.frameside = graphics.frameside*graphics.canvas_prop
			
	def dumpGraphics(self):

		if self.graphics != 0:
			self.id_frame 		= None
			self.displace 		= None
			self.wall_displace 	= None
			self.screen    		= None
			self.frameside 		= None
			
		self.graphics = 0
			
			
	def get_state(self): 
		return [self.q, self.x]
		
	def set_goal(self, q_goal):
		self.q_goal = q_goal
		
	def step(self, legend = '', reachout = None):
		pass
		
	def render(self, x_goal = None, legend = ''):
		pass
			
	def drawAgent(self):
		pass
			
			
	def drawCircle(self, x, col, rad = 4, width = 0):
		pos = self.displace + np.int32(x*self.frameside)
		pg.draw.circle(self.screen, col, pos, rad, width)
		
	def drawCross(self, x, col):
		pos = self.displace + np.int32(x*self.frameside)

		o1 = 5*np.array([1,1])
		o2 = 5*np.array([-1,1])
		
		pg.draw.line(self.screen, col, pos-o1, pos+o1, 4)
		pg.draw.line(self.screen, col, pos-o2, pos+o2, 4)
		
	def drawStar(self, x, col):
		pos = self.displace + np.int32(x*self.frameside)

		o1 = 6*np.array([np.sqrt(3)/2,.5])
		o2 = 6*np.array([-np.sqrt(3)/2,.5])
		o3 = 6*np.array([0,1])
		
		pg.draw.line(self.screen, col, pos-o1, pos+o1, 1)
		pg.draw.line(self.screen, col, pos-o2, pos+o2, 1)
		pg.draw.line(self.screen, col, pos-o3, pos+o3, 1)
		pg.draw.line(self.screen, col, pos+o1, pos-o1, 1)
		pg.draw.line(self.screen, col, pos+o2, pos-o2, 1)
		pg.draw.line(self.screen, col, pos+o3, pos-o3, 1)
		
		
	def	drawLine(self, xi, xj, col):
		pos_i = self.displace + np.int32(xi*self.frameside)
		pos_j = self.displace + np.int32(xj*self.frameside)
		pg.draw.line(self.screen, col, pos_i, pos_j, 2)


	def plotHS(self, HS = None, xG = None, hs_rad = None, P = None, legend = '', spherical = False):
		
		gph = self.graphics
		gph.draw_matrix(np.ones((1,1)), self.id_frame, v_min = 0, v_max = 1, matrix_text = legend)
	
		pxl = gph.frameside*gph.canvas_prop
		if HS is not None:
			HS_X = HS[1]
			
			# Show edges
			if P is not None:
				for i in range(len(HS_X)):
					xi = HS_X[i]
					for j in range(i):
						xj = HS_X[j]
						p = min(P[i,j], P[j,i])
						
						thresh = .1
						if p > thresh:
							prop = (p-thresh)/(1-thresh)
							c  = int(255*(1-prop))
							color = (255,c,c)
							
							if spherical:
								dist = abs(np.array([xi[0]-1,xi[0],xi[0]+1])-xj[0])
								iMin = np.argmin(dist)
								
								if iMin == 1:
									self.drawLine(xi,xj,col = color)
								elif iMin == 0:
									xi_m = copy.deepcopy(xi)
									xj_p = copy.deepcopy(xj)
									
									xi_m[0] = xi[0]-1
									xj_p[0] = xi[0]+1

									self.drawLine(xi_m,xj,  col = color)
									self.drawLine(xi,  xj_p,col = color)
								elif iMin == 2:
									xi_p = copy.deepcopy(xi)
									xj_m = copy.deepcopy(xj)
									
									xi_p[0] = xi[0]+1
									xj_m[0] = xi[0]-1
									
									self.drawLine(xi_p,xj,  col = color)
									self.drawLine(xi,  xj_m,col = color)
							else:
								self.drawLine(xi,xj,col = color)
				
			# Show nodes
			for xn in HS_X:
				self.drawCircle(xn, col = RED)
				
				if hs_rad is not None:
					pxlRad = int(hs_rad*pxl)
					self.drawCircle(xn, col = GRAY, rad = pxlRad, width = 2)
					
		if xG is not None:
			self.drawStar(xG,  col = BLUE)
		

	# See if there was a mouse click
	def check_events(self):
		gph = self.graphics
		
		for event in pg.event.get():
		
			#if user wants to quit
			if event.type == QUIT:
				#end the game and close the window
				pg.quit()
				sys.exit()
				
			elif event.type == MOUSEBUTTONDOWN:
				window_target = pg.mouse.get_pos()
				[frame_id, frame_pos] = gph.get_frame_and_pos(window_target)
				
				return gph.get_x_from_frame(frame_pos)
				
		return None
			
			
class Rooms(Parent):
	
	############### CREATE ENVIRONMENT ###############
	def __init__(self, random_start = False, q_start = None, nSteps = 500):
		super(Rooms, self).__init__()
		
		if q_start is None:
			self.q = self.get_random_posture()
		else:
			self.q = q_start
			
		self.x = self.q
		self.q_old = self.q
		self.x_old = self.x
		
		self.nSteps = nSteps	# nbr of steps to travel 1 unit
		
		self.walls  = Rooms4.getWalls()
		self.doors  = Rooms4.getDoors()
		
		self.render()
		
		
	# Get a random legal posture q
	def get_random_posture(self):
		return np.random.rand(2)
		
	def set_goal(self, q_goal):
		self.q_goal = q_goal
		self.x_goal = q_goal
		
		
	def step(self, legend = '', reachout = None):		
		self.q_old = self.q
		
		addUncert = True
		if addUncert:
			dist 	= np.linalg.norm(self.q - self.q_goal)
			q_goal	= self.q_goal + 0.02*dist*np.random.randn(len(self.q_goal)) 
			self.q 	= Rooms4.getNext(self.q, q_goal) 
		else:
			self.q = Rooms4.getNext(self.q, self.q_goal)
			
		self.x = self.q

		# Graphical stuff					
		self.drawWalls(self.walls, BLACK)
		self.drawWalls(self.doors, WHITE)
		
		if self.x_goal is not None:
			self.drawCross(self.x_goal, col = RED)
		
		q_old = self.q_old
		q_new = self.q
		dist  = np.linalg.norm(q_old - q_new)
		steps = int(dist*self.nSteps)

		q_inter = q_old		
		for p in np.linspace(0,1,steps):
			self.drawCircle(q_inter, col = WHITE)
			q_inter = p*q_new + (1-p)*q_old
			self.drawCircle(q_inter, col = BLUE)
			time.sleep(.001)
			pg.display.update()
		
		
	def drawWalls(self, walls, col = BLACK):
		for i in range(len(walls)):
			rect = self.wall_displace + walls[i]*self.frameside
			pg.draw.rect(self.screen, col, rect)
		
		

class Arm(Parent):
	
	############### CREATE ENVIRONMENT ###############
	def __init__(self, graphics = Graphics.Basic(), id_frame = 0, random_start = False, armLength = 1.0, 
					dof = 100, q_start = None, wall_setting = -1, nSteps = 20, maxStep = 1.):
		super(Arm, self).__init__(graphics, id_frame)
		
		# THE AGENT
		#  General settings
		self.dof 	= dof								# Degrees of freedom
		self.dim 	= [2,dof]							# [dim(task), dim(posture)]
		self.segLen = armLength/dof						# Length of an arm segment
		self.aMax   = np.pi								# Max angle on joint (<= pi)
		
		self.wall_setting	= wall_setting
		self.armLength		= armLength
		
		#  Start conditions - this is overwritten if 'random_start' or 'm_start' is given.
		aStart 			= 1.*4*np.pi/dof					
		self.angles 	= np.linspace(aStart,aStart/4,dof)	# Start in a spiral
		self.angle_old 	= self.angles
		self.angle_drawn= self.angles						# For graphics
		self.root 		= np.array([0.5, 0.5])				# Position of arm's base
		self.angleStart = copy.deepcopy(self.angles)
		self.angleGoal 	= np.zeros(dof)	# start by unfolding
		
		# THE UPDATES
		self.firstRound  = True
		self.updateSteps = nSteps
		self.stepsLeft	 = nSteps	# Number of steps of motion
		self.maxStep	 = maxStep	# How far a joint can travel in one transition (speedlimit)
		
		# THE ENVIRONMENT
		cW = 0.2  # corridor width
		wW = 0.02 # wall thickness
									
		self.walls 	= np.array([[0,0,0,0]])
		
		choice = wall_setting # Add walls
		if choice == 0:
			self.walls 	= np.array([[.2, .2, .2, .2]])
		elif choice == 1:
			self.walls 	= np.array([[ 1 - cW - wW, 2*cW, wW, 2*cW],
									[ 0.  , 4*cW, 1 - cW, wW]])
		elif choice == 2:
			cW = 1./6
			self.walls 	= np.array([[   cW,   1.5*cW, wW, 1 - 3*cW],
									[1-wW-cW, 1.5*cW, wW, 1 - 3*cW]])
		elif choice == 3:
			self.walls 	= np.array([[1.5*cW,   cW, 1 - 3*cW, wW],
									[1.5*cW, 1-wW-cW, 1 - 3*cW, wW],
									[   cW,   1.5*cW, wW, 1 - 3*cW],
									[1-wW-cW, 1.5*cW, wW, 1 - 3*cW]])
		elif choice == 4: # One u-shaped wall above
			self.walls 	= np.array([[ 1 - cW - wW, cW, wW, cW],
									[ cW, cW, 1 - 2*cW, wW],
									[ cW, cW, wW, cW]])
									
		elif choice == 5: # One u-shaped wall bellow
			self.walls 	= np.array([[          cW, 1 - cW - wW, 1 - 2*cW, wW],
									[ 1 - cW - wW,    1 - 2*cW,       wW, cW],
									[          cW,    1 - 2*cW,       wW, cW]])
									
		elif choice == 6: # Simple wall bellow
			self.walls 	= np.array([[2*cW, 1 - cW - wW, cW, wW]])
			
		elif choice == 7: # Long wall above
			self.walls 	= np.array([[ cW, 1-cW, 1 - 2*cW, wW]])
			
		elif choice == 8: # cross
			self.walls 	= np.array([[ .5-.5*wW,        0, wW, cW], 
									[ .5-.5*wW,     1-cW, wW, cW],
									[ 0       , .5-.5*wW, cW, wW],
									[ 1-cW    , .5-.5*wW, cW, wW]])

		# INITIAL UPDATE
		if random_start:
			q_start = self.get_random_posture()	

			self.angles = self.q2ang(q_start)
			
		elif q_start is not None:
			self.angles = self.q2ang(q_start)

		else: # Move arm until it is stopped
			self.step()
			
		self.firstRound = False
		
	############### CREATE ENVIRONMENT - DONE ###############

	
	## PUBLIC METHODS ##
	#  Set goal-posture
	def set_goal(self, q_goal):
		self.angleGoal = self.q2ang(q_goal)
		self.stepsLeft = self.updateSteps
		
	#  Update graphics
	#  Input:
	#	- x_goal 		Where the planner wants the end-effector to move next
	#	- background	A matrix in the background, for example the V-function
	def render(self, x_goal = None, x_long_goal = None, background = None, legend = ''):
		gph = self.graphics
		
		# Full update of background
		if False: #self.stepsLeft == self.updateSteps or background is not None:
			if background is None:
				gph.draw_matrix(np.ones((1,1)), self.id_frame, v_min = 0, v_max = 1, matrix_text = legend)
			
			else:
				v_min = np.min(background)
				v_max = np.max(background)
				v_min = 2*v_min - v_max
				gph.draw_matrix(background, self.id_frame, v_min = v_min, v_max = v_max, matrix_text = legend)				
			
			self.drawWalls()
		else:
			self.drawAgent(self.angle_drawn, WHITE) # Paint over old
			
		# Draw arm and goal
		self.drawWalls()
		self.drawAgent(self.angles, BLACK)
		
		self.angle_drawn = self.angles
		
		if x_goal is not None:
			self.drawCircle(x_goal, RED)
			
		if x_long_goal is not None:
			self.drawCross(x_long_goal, BLUE)
			
		pg.display.update()
		
		#self.check_events()

	
	def step(self, x_goal = None, background = None, legend = '', reachout = None):
		while True:
			self.render(x_goal = x_goal, background = background, legend = legend)
			observe, done, stopped = self.update()
			
			if done:
				self.render()
				break
				
		return not stopped
		
					
	#  Take one step in environment
	#  Return:
	#	- observation = [x,m] 		Current position in sensor- and posture-space
	#   - done		  = True/False	Are there more updates before reaching goal?
	#	- stopped     = True/False	Was the movement stopped due to a collision (or too high speed)
	def update(self):
		stopped = False
		
		stepsLeft	= self.stepsLeft
		angle_now  	= self.angles
		angle_goal 	= self.angleGoal
		
		if stepsLeft > 0:
			prop = 1./stepsLeft
			
			angle_next = (1.-prop)*angle_now + prop*angle_goal
			
			# If the update can be made
			if self.isLegal(self.ang2q(angle_next), self.ang2q(angle_now)):
				self.angles 	= angle_next
				self.stepsLeft -= 1
				
			else:
				stopped			= True
				self.stepsLeft	= 0
				
		q = self.ang2q(self.angles)
		x = self.q2x(q)
		
		self.angle_old = angle_now
		
		observation = self.get_state()
		done 		= (self.stepsLeft == 0)
		
		return observation, done, stopped
		
		
	def get_state(self): 
		q = self.ang2q(self.angles)
		x = self.q2x(q)
		
		return [q, x]

	
	## PRIVATE METHODS ##
	# If a session is loaded new graphics need to be created to show graphics
	def add_graphics(self, graphics, id_frame):
		self.graphics		= graphics 
		self.displace 		= np.array(graphics.get_canvas_displacement(id_frame))	# This is where the plotting starts
		#self.wall_displace 	= np.append(self.displace , np.array([0,0]))
		self.screen    		= graphics.screen
		self.frameside 		= graphics.frameside*graphics.canvas_prop
		
	# To save an environment graphics must be discarded
	def remove_graphics(self):
		self.graphics		= None
		self.displace 		= None
		#self.wall_displace 	= None
		self.screen    		= None
		self.frameside 		= None
		
	# Convert radians to posture q
	def ang2q(self, ang):
		return 1.*ang / (self.aMax*2) + 0.5	# Change span from [-a,a] to [0,1]
		
	# Convert posture q to radians
	def q2ang(self, q):
		return (2.*q - 1)*self.aMax		# Change span from [0,1] to [-a,a]
		
	# Get x from q - Basically the forward model
	def q2x(self, q):
		coord  = self.getCoordinates(self.q2ang(q))

		return coord[-1,:]
		
	# Transform angles of arm to coordinates of joints
	def getCoordinates(self, angles):
		coordX = np.zeros(self.dof + 1) + self.root[0]
		coordY = np.zeros(self.dof + 1) + self.root[1]
		tmpAng = np.pi/2	#start angle
		
		for i in range(self.dof):
			tmpAng += angles[i]
			
			coordX[i+1] = coordX[i] + self.segLen*np.cos(tmpAng)
			coordY[i+1] = coordY[i] + self.segLen*np.sin(tmpAng)
			
		return np.array([coordX, coordY]).T
		
	# Get a random legal posture q
	def get_random_posture(self):
		while True:
			a_try = np.random.randn(self.dof).clip(-self.aMax, self.aMax)
			q_try = self.ang2q(a_try)
			
			if self.isLegal(q_try, q_try):
				return q_try
				
	# Check that arm doesn't break the physics of the simulation.
	# 1: It can't move any joint a distance > maxStep in one transition
	# 2: It can't pass through walls, or move out of the arena
	# 3: The arm can't intersect itself.
	def isLegal(self, q, q_prev):
		joint_coord = self.getCoordinates(self.q2ang(q))
		
		# TEST 1: Would any joint travel too far?
		if not self.firstRound:
			joint_coord0 = self.getCoordinates(self.q2ang(q_prev))
				
			# Modified to only check end effector (for speed)
			for i in range(len(joint_coord) - 1, len(joint_coord)):
				dist = np.linalg.norm(joint_coord[i] - joint_coord0[i])
				
				if dist > self.maxStep/self.updateSteps:
					#print('Too fast')
					return False
		
		# TEST 2a: Is the arm within boundaries?
		if np.min(joint_coord) < 0 or np.max(joint_coord) >= 1:
			#print('Outside of arena')
			return False
			
		# TEST 2b: Is the arm within or crossing any walls?
		#  If dof is too low points will be too far away for collision test
		segLen = self.segLen
		
		if len(joint_coord) < 101:
			new_coord = np.zeros((101, 2))
			new_coord[:,0] = np.interp(np.linspace(0,1,101),\
										np.linspace(0,1,len(joint_coord)), joint_coord[:,0])
			new_coord[:,1] = np.interp(np.linspace(0,1,101),\
										np.linspace(0,1,len(joint_coord)), joint_coord[:,1])
										
			segLen *= self.dof*1./101
			
			joint_coord = new_coord
		
		# Would it go through walls?
		walls = self.walls
		
		for i in range(len(walls)):
			for j in range(len(joint_coord)):
				x  = joint_coord[j,0]
				y  = joint_coord[j,1]
				x1 = walls[i,0]
				x2 = x1 + walls[i,2]
				y1 = walls[i,1]
				y2 = y1 + walls[i,3]
				
				if x1 < x and x < x2 and y1 < y and y < y2:
					#print('Wall collision!')
					return False
		
		
		# TEST 3: Would it go through itself?
		#  I will do this simpler by drawing a circle around every segment and see if two segments
		#  intersect
		radius = segLen*0.95
		
		# Sort by x for faster clasification
		joint_coord = joint_coord[ joint_coord[:,0].argsort() ]

		for i in range(len(joint_coord)):
			for j in range(i+1, len(joint_coord)):
				dx = np.abs(joint_coord[i,0] - joint_coord[j,0])
				dy = np.abs(joint_coord[i,1] - joint_coord[j,1])
				
				#print([i,j,dx, radius])
				if dx > radius:
					break
				
				if dy + dx < radius:
					#print('Self collision')
					return False
		
		return True

		
	# GRAPHICAL METHODS	
	def drawAgent(self, angles, col, line_width = LINE_WIDTH):
		# this will scale and draw a line between the coordinates
		# col is the color of the lines
		
		coord = self.getCoordinates(angles)
		pg.draw.lines(self.screen, col, False, self.displace + coord*self.frameside, line_width)

	def drawWalls(self):
		walls = self.walls

		for i in range(len(walls)):
			rect = self.wall_displace + walls[i]*self.frameside
			pg.draw.rect(self.screen, BLACK, rect)


class Nao(Parent):
	
	############### CREATE ENVIRONMENT ###############
	def __init__(self, graphics = Graphics.Basic(), id_frame = 0, ip_address = None, 
														stiffHS = None, stiffReach = None):
		super(Nao, self).__init__(graphics, id_frame)
		
		if ip_address is None:
			ip_address = str(raw_input('Enter IP_ADDRESS: '))
		
		IP_ADRESS 	= ip_address
		PORT		= 9559
		
		prox = qi.Session()
		prox.connect("tcp://" + IP_ADRESS + ":" + str(PORT))
			
		self.body = BodyNao.Body(prox)
		
		self.body.relax()
		time.sleep(4)
		x = self.body.get_x()
		q = self.body.get_q()
		
		self.x 		= x
		self.q 		= q
		self.x_old	= copy.deepcopy(x)
		self.q_old	= copy.deepcopy(q)
		self.q_goal = copy.deepcopy(q)
		
		if stiffHS is None:
			stiffHS = body.STIFFNESS
		if stiffReach is None:
			stiffReach = body.STIFFNESS
		
		self.stiffReach = stiffReach
		self.stiffHS	= stiffHS
		
		
	# Transform from spherical coord (2D) (norm to [0,1]) to cartees (3D) (of rad=1)
	def sph2car(self, sph):
		phi = sph[0]*2*np.pi
		the = sph[1]*np.pi
		
		x = np.sin(the)*np.cos(phi)
		y = np.sin(the)*np.sin(phi)
		z = np.cos(the)
		
		return np.array([x,y,z])
		
	def car2sph(self, car):
		x = car[0]
		y = car[1]
		z = car[2]
		
		the = np.arccos(z)/np.pi
		phi = np.arctan2(y,x)/(2*np.pi)
		phi = np.mod(phi, 1)	# [-.5, .5] to [0,1]
		
		return np.array([phi, the])
		
		
	# See if there was a mouse click
	def check_events(self):
		sph = super(Nao, self).check_events()
		
		if sph is not None:
			return self.sph2car(sph)
			
		return None
		
		
	# Plot in spherical coord
	def plotHS(self, HS = None, xG = None, hs_rad = None, P = None, legend = ''):
		
		if HS is not None:
			sphHS = copy.deepcopy(HS)
			
			for i in range(len(sphHS[1])):
				sphHS[1][i] = self.car2sph(sphHS[1][i])

			
		if xG is not None:
			sphG = self.car2sph(xG)
		super(Nao, self).plotHS(sphHS, sphG, None, P, legend, spherical = True)
		
		
	def step(self, reachout):	
		body = self.body
		body.set_goal(self.q_goal)
		
		if reachout:
			x,q = body.step(self.stiffReach)
		else:
			x,q = body.step(self.stiffHS)
		
		self.x_old	= self.x
		self.q_old	= self.q
		self.x 		= x
		self.q 		= q
		
		sph_old = self.car2sph(self.x_old)
		sph_new = self.car2sph(x)
		
		self.drawLine(sph_old,sph_new, col = BLACK)
		self.drawCircle(sph_old, col = WHITE)
		self.drawCircle(sph_new, col = BLACK)
		pg.display.update()
		
		

if False:
	agent = Rooms(speed = 600) #Arm() #
	for i in range(100):
		[x0, q0]  = agent.get_state()	# Get current state
		q_goal = (q0 + 0.2*np.random.randn(*q0.shape)).clip(0,1)
		agent.set_goal(q_goal)
		
		agent.step(legend = str(i))
