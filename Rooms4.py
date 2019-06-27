# 4 Rooms
import numpy as np
import matplotlib.pyplot as plt
import pygame as pg

doorThick	= 0.01
doorRad 	= 0.05
doorsH		= [1./4, 3./4]
doorsV  	= [1./4]

xWall = 0.5
yWall = 0.5

# See where you ended up
def getNext(s_i, s_g):
	
	if s_g.ndim == 2:
		S_f = np.zeros(s_g.shape)
		for i in range(len(s_g)):
			S_f[i] = getNext(s_i, s_g[i])
			
		return S_f
	
	# Did we cross vertical wall?
	collide, s_f = check_wall(s_i, s_g, xWall, 0, doorsV)
	if collide:
		return getNext(s_i, s_f)
	
	# Did we cross horisontal wall?
	collide, s_f = check_wall(s_i, s_g, yWall, 1, doorsH)
	if collide:
		return getNext(s_i, s_f)
		
	# Did we move outside of arena?
	collide, s_f = check_wall(s_i, s_g, 0, 0)
	if collide:
		return getNext(s_i, s_f)
	collide, s_f = check_wall(s_i, s_g, 0, 1)
	if collide:
		return getNext(s_i, s_f)
	collide, s_f = check_wall(s_i, s_g, 1, 0)
	if collide:
		return getNext(s_i, s_f)
	collide, s_f = check_wall(s_i, s_g, 1, 1)
	if collide:
		return getNext(s_i, s_f)
			
	if np.abs(s_g[0] - xWall) < doorThick:
		s_g[0] += np.sign(s_i[0]-s_g[0])*doorThick
		
	if np.abs(s_g[1] - yWall) < doorThick:
		s_g[1] += np.sign(s_i[1]-s_g[1])*doorThick
		
	return s_g.clip(0,1)
	
def plotRoom():
	plt.plot([0,0],[0,1], 'k', linewidth = 2)
	plt.plot([0,1],[1,1], 'k', linewidth = 2)
	plt.plot([1,1],[1,0], 'k', linewidth = 2)
	plt.plot([1,0],[0,0], 'k', linewidth = 2)
	plt.plot([0,1],[yWall,yWall], 'k', linewidth = 2)
	plt.plot([xWall,xWall],[0,1], 'k', linewidth = 2)
	for door in doorsH:
		plt.plot([door-doorRad,door+doorRad],[yWall,yWall], 'w', linewidth = 4)
	
	for door in doorsV:
		plt.plot([xWall,xWall], [door-doorRad,door+doorRad], 'w', linewidth = 4)
		
def getWalls():
	walls = np.array([	[ -0.01, yWall-doorThick, 1+.02, 2*doorThick],
						[ xWall-doorThick, -0.01, 2*doorThick, 1+.02]	])
	return walls
	
def getDoors():
	doors = []
	for d in doorsH:
		door = [d-doorRad, yWall-doorThick, 2*doorRad, 2*doorThick]
		doors.append(door)
		
	for d in doorsV:
		door = [xWall-doorThick, d-doorRad, 2*doorThick, 2*doorRad]
		doors.append(door)
	
	return np.array(doors)
		
def plotRoomGraph(gph):
	plt.plot([0,0],[0,1], 'k', linewidth = 2)
	plt.plot([0,1],[1,1], 'k', linewidth = 2)
	plt.plot([1,1],[1,0], 'k', linewidth = 2)
	plt.plot([1,0],[0,0], 'k', linewidth = 2)
	plt.plot([0,1],[yWall,yWall], 'k', linewidth = 2)
	plt.plot([xWall,xWall],[0,1], 'k', linewidth = 2)
	for door in doorsH:
		plt.plot([door-doorRad,door+doorRad],[yWall,yWall], 'w', linewidth = 4)
	
	for door in doorsV:
		plt.plot([xWall,xWall], [door-doorRad,door+doorRad], 'w', linewidth = 4)

# Check wall collision
# posWall: position along dim-axis
def check_wall(s_i, s_g, posWall, dim, doors = []):
	collide = False
	s_new   = None
	
	
	if 	(s_i[dim] < posWall and s_g[dim] > posWall) or \
		(s_i[dim] > posWall and s_g[dim] < posWall):
		# At what y did it cross?
		dx  = s_g[dim] - s_i[dim]
		dx1 = posWall - s_i[dim]
		
		dim2 = np.mod(dim+1,2)
		dy  = s_g[dim2] - s_i[dim2]
		dy1 = dy*dx1/dx
		
		y = s_i[dim2] + dy1
		
		collide = True
		for door in doors:
			if abs(y - door) <= doorRad:
				collide = False
				continue
				
		if collide:
			s_new = np.zeros(2)
			s_new[dim]  = posWall
			s_new[dim2] = y

	return collide, s_new


test = False
if test:
	plotRoom()
	s = [np.random.rand(2),np.random.rand(2),np.random.rand(2),np.random.rand(2)]
	c = ['g','r','b','c']

	for i in range(500):
		index = np.random.randint(len(s))
		s_i = s[index]
		s_g = 2*np.random.rand(2)-0.5
		s_f = getNext(s_i,s_g)

		line = np.array([s_i,s_f]).T
		plt.plot(line[0], line[1], color = c[index])
		
		#plt.plot(s_f[0],s_f[1], '.', color = c[index])
		plt.xlim(-0.1,1.1)
		plt.ylim(-0.1,1.1)
	plt.show()
