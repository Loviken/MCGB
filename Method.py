# Method - Multi Core Goal Babbling
#
# s = [q, x1, x2, ...]
# q  - posture
# xi - task space position in space Xi

import numpy as np
import copy

class Plain:
	
	def __init__(self, s_start, stateThresh	= 0.01, regionRad = 0.2, weightP0 = False, exploreRate = 0.1, 
													exploreMax = 0.1, homeStateHack = True):
		self.s  = s_start		# current state
		self.n  = 0				# Current node
		
		self.xG 		= np.random.rand(len(s_start[1]))	# What goal to reach
		self.goalSpace	= 0									# In what task space to reach it
		
		self.stateThresh	= stateThresh	# How similar it must be to be the same state
		self.regionRad		= regionRad		# How far one can reach from a home state
		self.weightP0       = weightP0		# Should P <- 1 or depend on distance?
		self.exploreRate	= exploreRate
		self.exploreMax		= exploreMax
		self.homeStateHack  = homeStateHack	# Sometimes use hs instead of nearest sample when reaching
		
		# MDP
		self.P = {}						# prob to move between homestates
		self.R = None					# reward to reach state
		self.Q = None
		self.V = None
		self.gamma = .95				# Future reward decay
		self.alpha = 0.5				# Probability update
		self.nBranch = 2				# Branch to x closest, when new hs.	
		
		self.P_reach = []				# Probability that a new home state is reached
		
		# Home States
		self.HS = []					# home states
		for i in range(len(s_start)):	# adjust to number of task spaces
			self.HS.append([])
			
		self.DataSets = []
			
		# Init
		self.newHomeState(s_start)
		
		
	def newHomeState(self, s_new):
		for i in range(len(self.HS)):
			self.HS[i].append(s_new[i])	# Add appropriate space
			
		nHS = len(self.HS[0])
		
		# Update Dataset
		self.DataSets.append([])
		self.addToData(nHS-1, s_new, s_new)
		
		# Update MDP
		dist = []
		for n in range(nHS):
			dist.append(np.linalg.norm(self.getState(n)[1] - s_new[1]))
			
			# initialize
			self.P[n,nHS-1] = 0	
			self.P[nHS-1,n] = 0	
			
		N_min = np.argsort(dist)

		for i in range(min(self.nBranch+1, nHS)):
			self.P[N_min[i],nHS-1] = 1.	
			self.P[nHS-1,N_min[i]] = 1.	
			
		# Assume transition it was found through is possible
		self.P[self.n,nHS-1] = 1.
		self.P[nHS-1,self.n] = 1.
		
		self.P_reach.append(1.)	# Assume anything can be reached from state
		self.updateReward()
		
	def replaceHomeState(self, n, s):
		for i in range(len(self.HS)):
			self.HS[i][n] = s[i]	# Add appropriate space
			
		self.DataSets[n] = []
		self.addToData(n, s, s)
		
		# Update MDP
		nHS  = len(self.HS[0])	# number of home-states
		dist = []

		for nn in range(nHS):
			dist.append(np.linalg.norm(self.getState(nn)[1] - self.getState(n)[1]))
			
			# initialize
			'''
			self.P[nn,n] = 0	
			self.P[n,nn] = 0
			'''	
			
		N_min = np.argsort(dist)

		for i in range(min(self.nBranch+1, nHS)):
			self.P[N_min[i],n] = 1.
			self.P[n,N_min[i]] = 1.
			
		# Assume transition it was found through is possible
		self.P[self.n,n] = 1.
		self.P[n,self.n] = 1.
		
		self.P_reach.append(1.)	# Assume anything can be reached from state
		self.updateReward()
		
		
	# I don't think this is used
	def update(self, n_new, s_new):
		HS = self.HS
		uRate = 0.9	# Update rate
		for i in range(len(HS)):
			HS[i][n_new] = uRate*HS[i][n_new] + (1 - uRate)*s_new[i]
		
		
	def getState(self, n):
		s = []
		for i in range(len(self.HS)):
			s.append(self.HS[i][n])
			
		return s
		
		
	def stateDiff(self, s_m, s_n):
		diff = 0
		for i in range(len(s_m)):
			diff = max(diff, np.max(abs(s_m[i]-s_n[i])))
			
		#print np.mean(abs(s_m[0]-s_n[0]))
		return diff
		
			
	def updateReward(self, xG_new = None, goalSpace = None):
		if xG_new is not None:
			self.xG = xG_new
			
		if goalSpace is not None:
			self.goalSpace 	= goalSpace
			#for p in self.P_reach:
			#	p = 1.
		
		nHS 	= len(self.HS[0])
		Reward 	= np.zeros(nHS)
		
		for n in range(nHS):
			dist 		= np.linalg.norm(self.xG - self.HS[self.goalSpace+1][n])
			
			if dist < self.regionRad:
				Reward[n] = 1
			else:
				# Must be exponential so infinite chain of states with discounted future reward can be better
				Reward[n] = 0.01*np.exp(-dist/self.regionRad)	
				
			
		self.R = Reward
		self.updateQV()
		
	# Make P into ordinary array
	def getP(self):
		nHS   = len(self.HS[0])
		
		P     = np.zeros((nHS, nHS))
		for i in range(nHS):
			for j in range(nHS):
				P[i,j] = self.P[i,j] # Make into array
				
		return P
		
		
	def updateQV(self):
		nHS   = len(self.HS[0])
		
		Q_new = np.zeros((nHS, nHS))
		V_new = np.zeros(nHS)
		gamma = self.gamma
		
		P     = self.getP()
		
		
		# VALUE ITERATION
		iterations = 0	# If it doesn't converge (shouldn't be possible)
		while True:
			Q_old  = Q_new
			Q_new  = P*(self.R + gamma*V_new)
			diff   = np.max(np.abs(Q_new - Q_old))
			
			V_new = np.max(Q_new, axis=1)

			if diff < 10**(-5):
				break
			
			iterations += 1 
			if iterations > 10**5:
				print('V and Q couldn\'t converge!')
				break
				
		self.V, self.Q = V_new, Q_new
		
	def getGoal_n(self):
		return np.argmax(self.Q[self.n])
		
	def getGoal_s(self, n):
		DS 		= self.DataSets[n] # Get the dataset
		
		X_New  = []
		S_Goal = []
		
		for ds in DS:
			S_Goal.append(ds[0])
			X_New.append(ds[1][1])
			
		X    = np.array(X_New)
		dist = np.linalg.norm(X-self.xG, axis=1)
		iMin = np.argmin(dist)
		
		# Use home posture sometime (2* = half the time if equally close)
		if self.homeStateHack and 2*np.random.rand() < dist[iMin]/dist[0]:
			#print 'Use home state, ratio: ' + str(dist[iMin]/dist[0])
			iMin = 0
		
		d = min(self.exploreMax, dist[iMin])
		
		s_g     = copy.deepcopy(S_Goal[iMin])
		
		noise	= self.exploreRate*d*np.random.randn()*np.random.randn(*s_g[0].shape)
		s_g[0] += noise
		s_g[0]  = s_g[0].clip(0,1)
		
		s_g[1]  = self.xG
		
		return s_g
		
		
	def addToData(self, n, s_goal, s_new):
		self.DataSets[n].append([s_goal, s_new])
	
	def observe(self, n, n_goal, n_new):
		alph = self.alpha
		
		#if n_goal != n_new
		
		if n != n_goal:	# Move within digraph
			self.P[n, n_goal] = alph*self.P[n, n_goal] + (1-alph)*int(n_goal == n_new)
		else:
			self.P_reach[n]   = alph*self.P_reach[n] + (1-alph)*int(n_goal != n_new) #not using P_reach...
			
		self.updateReward()

	def getReachables(self, thresh):
		nHS 	  = len(self.HS[0])
		reachable = [False]*nHS
		
		nStart = self.n
		reachable = self.findReachable(reachable, nStart, thresh)
		
		return reachable
		
	
	def findReachable(self, reachable, n0, thresh):
		
		reachable[n0] = True
		for n in range(len(reachable)):
			if not reachable[n] and self.P[n0, n] > thresh:
				reachable = self.findReachable(reachable, n, thresh)
				
		return reachable


if False:
	q = np.random.rand(2)
	x = q
	method = Plain([q,x])
	
	print method.getP()
