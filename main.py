import Environment
import Method
import Data
import numpy as np
import time
import cPickle as pickle
import copy
import Graphics


def StateRecovery(s_new, n_goal, env, meth, iteration = 0, depth = 0, record = None):
	nHS = len(meth.HS[1])
	# Compute distance to all HS
	x_new  = s_new[1]
	diff   = np.zeros(nHS)
	
	thresh    = 0.1	# A node is reahable if prob greater than this to reach from other reachable
	reachable = meth.getReachables(thresh)
	#print reachable
	
	for n in range(nHS):
		x_n     = meth.HS[1][n]
		diff[n] = np.linalg.norm(x_n - x_new)
		
		if not reachable[n] and n != n_goal: # Ignore unreachable neighbors
			diff[n] = 100 # Hack to allow new states in regions of unreachable states
			#print reachable
	
	# Did we reach it?
	if diff[n_goal] < meth.stateThresh:
		return [s_new, n_goal]
		
	# If not, try recover, unless too deep, or if a new area is discovered
	N_min = np.argsort(diff)	# order of closeness
	
	if diff[N_min[0]] < meth.regionRad and depth < 4:
		if depth == 0:
			n_goal = n_goal 	# Try again, or return if in reachout
		else:
			print 'diff: ' + str(diff[N_min[0]])
			n_goal = N_min[0] #meth.n
		#else:
		#	n_goal = N_min[min(depth-2, nHS-1)]	# Go to closest
			
		s_goal = meth.getState(n_goal)
		
		if record is not None:
			activity = 2	# recover
			record.motions.append([iteration, activity, [meth.n, s_new], [n_goal,s_goal]])
		
		s_new = goTo(s_goal, env, meth, legend = str(iteration))
		
		return StateRecovery(s_new, n_goal, env, meth, iteration, depth = depth+1, record = record)
	
	dontMove = False
	
	if diff[N_min[0]] > meth.regionRad:
		n_new = len(meth.HS[0])
		meth.newHomeState(s_new)
	
		if record is not None:
			record.home_states.append([iteration, s_new, 0]) # 0 = New region
			
	else:
		if record is not None:
				record.home_states.append([iteration, s_new, 1]) # 1 = Unable to return
		
		if dontMove:	# Add new
			n_new = len(meth.HS[0])
			meth.newHomeState(s_new)
		else:			# Replace closest reachable
			n_new = N_min[0]
			meth.replaceHomeState(n_new,s_new)
	
	return [s_new, n_new]


def goTo(s_goal, env, meth, legend = '', reachout = None):
		
	env.set_goal(s_goal[0])
	env.plotHS(HS = meth.HS, xG = meth.xG, hs_rad = meth.regionRad, P = meth.getP(), legend = legend)
	env.step(reachout = reachout)
	
	ev = env.check_events()
	if ev is not None:
		env.x_Glob = ev
	
	return env.get_state()	# Get current state


####################################################
# iAgent:
# 0 - 4_Rooms
# 1 - 100_DoF_Arm
# 2 - Nao

run	   = 1
iAgent = 1
iRobot = 1		#if iAgent = 2	

testMode = False # Test saved file?

radius 		 = 0.2 # 0.2, 0.3, 0.4, 2.0
stiffness	 = 0.5
purgeUnreach = False

if iRobot == 1:
	save_name = 'Quasimodo'
else:
	save_name = 'Klumpfot'
	
	
load_name = None #  save_name # 
if testMode:
	save_name += '_1000it_test_cleanBackground'

if load_name is None:
	# Parameters
	#run 			= 1
	iterations		= [10000, 2000, 250]	# Number of iterations per run
	stateThresh		= [0.01, 0.001, 0.1]		# How similar it must be to be the same state
	regionRad		= [radius, radius, radius]		# How far one can reach from a home state
	exploreRate		= [0.5, 0.1, 2.0] 		# Noise variance (1.0)
	exploreMax		= [0.4, 0.2, 0.2]		# Max dist allowed for noice
	maxTry 			= [100, 20, 10]
	avarageStates	= False # not implemented
	weightP0		= True	# Should P(n1,n2) initialize dependent on distance?
	addDatasets		= True
	useReachAsGoal	= False	# Use motor command of where you ended up, not what you tried
	#purgeUnreach	= False	# Allow new homestates if only unreachable around. 
	distribHack		= False	# For Nao, sample more towards rotation
	homeStateHack	= False	# When reaching, sometimes use hs instead of nearest sample


	# Init
	record = Data.Record()	# Keep record on what the robot is doing. Set None to skip
	
	#np.random.seed(0)
	if iAgent == 0:
		env  = Environment.Rooms(nSteps = 1000)
	elif iAgent== 1:
		env  = Environment.Arm(Graphics.Basic(), wall_setting = 8, armLength = 1.0, nSteps = 20, dof=100)
	elif iAgent== 2:
		#iRobot = ?
		if iRobot == 1:
			IP_ADDRESS 	= '192.168.0.103'	# orange1
		elif iRobot == 2:
			IP_ADDRESS 	= '192.168.0.102'	# orange2
		elif iRobot == 3:									
			IP_ADDRESS 	= '192.168.43.184'	# orange3
		elif iRobot == 4:
			IP_ADDRESS 	= '192.168.0.104'	# cyan1
		elif iRobot == 5:
			IP_ADDRESS 	= '192.168.0.101'	# cyan2
		
		stiffHS		= 1.	# Stiffness when moving between states 
		stiffReach	= stiffness	# Stiffness when reaching out
		env 	 	= Environment.Nao(ip_address = IP_ADDRESS, stiffHS = stiffHS, stiffReach = stiffReach)
		
	if record is not None:
		record.home_states.append([0, env.get_state(), 0]) # second 0 = New region
		
	# initialize method
	meth = Method.Plain(env.get_state(), stateThresh[iAgent], regionRad[iAgent], weightP0, 
												exploreRate[iAgent], exploreMax[iAgent], homeStateHack)
												
	
else: # Load file
	pkl_file = open(load_name + '.pkl', 'rb') 
	data  	 = pickle.load(pkl_file)
	
	# Aquire record of previous evolution
	record = data[0]
	
	# Extract parameters
	parameters 		= data[1]
	
	iAgent 			= parameters[0]
	run 			= parameters[1]+1
	iterations		= parameters[2]
	stateThresh		= parameters[3]
	regionRad		= parameters[4]
	exploreRate		= parameters[5]
	exploreMax		= parameters[6]
	maxTry 			= parameters[7]
	avarageStates	= parameters[8]
	weightP0		= parameters[9]
	addDatasets		= parameters[10]
	useReachAsGoal	= parameters[11]
	purgeUnreach	= parameters[12]
	distribHack		= parameters[13]
	homeStateHack	= parameters[14]
	
	# Recreate environment
	agentInfo = data[2]
	if iAgent == 0:
		env  = Environment.Rooms(q_start = agentInfo[0], nSteps = agentInfo[1])
	elif iAgent== 1:
		env  = Environment.Arm(q_start = agentInfo[0], wall_setting = agentInfo[1], armLength = agentInfo[2])
	elif iAgent == 2: # Necessary to reinitiate
		
		iRobot = agentInfo[0] # If same physical robot
		if iRobot == 1:
			IP_ADDRESS 	= '192.168.0.103'	# orange1	
		elif iRobot == 2:
			IP_ADDRESS 	= '192.168.0.102'	# orange2	
		elif iRobot == 3:									
			IP_ADDRESS 	= '192.168.43.184'	# orange3
		elif iRobot == 4:
			IP_ADDRESS 	= '192.168.0.104'	# cyan1
		elif iRobot == 5:
			IP_ADDRESS 	= '192.168.0.101'	# cyan2
		
		stiffHS		= agentInfo[1]
		stiffReach	= agentInfo[2]
		env = Environment.Nao(ip_address = IP_ADDRESS, stiffHS = stiffHS, stiffReach = stiffReach)
	
	# Get method framework
	meth = data[3]
	meth.s = env.get_state()
	

''' Show postures while relaxed
float_formatter = lambda x: "%.2f" % x
np.set_printoptions(formatter={'float_kind':float_formatter})
while True:
	print np.array([range(26),env.body.get_q()]).reshape(2,-1).transpose()
	time.sleep(.5)
#'''
''' Complient joints test
env.body.motion_service.setStiffnesses("Body", 1.)
float_formatter = lambda x: "%.2f" % x
np.set_printoptions(formatter={'float_kind':float_formatter})
while True:
	# Read posture
	posture = env.body.get_q()
	print np.array([range(26),posture]).reshape(2,-1).transpose()
	
	# Set read posture as goal posture and tell robot to keep it
	env.body.set_goal(posture)
	env.body.motion_service.setAngles("Body", env.body.angle_goal.tolist(),.4)
	
	#time.sleep(.1)
#'''

							
if iAgent == 2:
	xG    = np.random.rand(3) - .5
	xG /= np.linalg.norm(xG)
	meth.updateReward(xG)

nTry   = 0
s_new,n_new  = meth.s, meth.n

######################## START ###########################
iStart = (run-1)*iterations[iAgent]
if testMode:
	iEnd = iStart + 50
else:
	iEnd   = 1000 # run	*iterations[iAgent]

for i in range(iStart, iEnd):
	
	if env.x_Glob is not None:
		meth.updateReward(env.x_Glob)
		env.x_Glob = None
		nTry   = 0
	
	s,n 			= s_new,n_new
	meth.s, meth.n 	= s,n
	
	n_goal = meth.getGoal_n()
	
	if n_goal != n: # Move in graph
		s_goal = meth.getState(n_goal)
		
		reachout = False
		
	else: # Reach for new
		s_goal = meth.getGoal_s(n)
		#s_goal[0] = env.body.get_random_q()
		
		reachout = True
		
	
	if record is not None:
		record.motions.append([i, int(reachout), [n,s], [n_goal,s_goal]])
		
	s_new = goTo(s_goal, env, meth, legend = str(i), reachout = reachout)
	#n_new = n_goal

	if addDatasets and reachout:
		if useReachAsGoal:
			meth.addToData(n,s_new, s_new)
		else:
			meth.addToData(n,s_goal, s_new)
		
	[s_new, n_new] = StateRecovery(s_new, n_goal, env, meth, iteration = i, record = record)
	
	print 'Iteration:' + str(i) + ' - ' + str(n) + '->' + str(n_new) + ' (' + str(n_goal) + ')'
	meth.observe(n, n_goal, n_new)
	
	if purgeUnreach:
		thresh = 0.05	# Prob to reach must be this at least...
		meth.purgeUnreach(thresh)	# Remove states that cannot be reached
	
	if record is not None:
		P = copy.deepcopy(meth.P)
		R = copy.deepcopy(meth.R)
		V = copy.deepcopy(meth.V)
		Q = copy.deepcopy(meth.Q)
		
		record.MDP.append([P,R,V,Q])
	
	
	#if reachout:
	nTry += 1
		
	if (nTry == maxTry[iAgent] or meth.R[n] == 1) and not testMode:
		if iAgent == 2:
			xG  = np.random.rand(3) - .5
			xG /= np.linalg.norm(xG)
			
			if distribHack:
				xG[2] = xG[2]**3
				xG /= np.linalg.norm(xG)
			
			meth.updateReward(xG)
		else:
			meth.updateReward(np.random.rand(2))
		nTry   = 0
		
	#print str(i) + ': ' + str(len(meth.P_reach))
	
for hs in record.home_states:
	print [hs[0], hs[1][1], hs[2]]

# Save model?
if save_name is not None:
	#filename = save_name + '_it' + str(run*iterations[iAgent]) + '.pkl'
	filename = save_name + '.pkl'
	y_or_n 	 = raw_input('Save robot as \'' + filename + '\'?  (y/n) :')

	if y_or_n == 'y':
		output 	 = open(filename, 'wb')
		
		parameters = [	iAgent, 		run, 			iterations, 	stateThresh, 	regionRad,
						exploreRate,	exploreMax, 	maxTry, 		avarageStates,	weightP0,
						addDatasets,	useReachAsGoal,	purgeUnreach,	distribHack, 	homeStateHack	]
		
		if iAgent == 0:
			agentInfo = [env.q, env.nSteps]
		elif iAgent == 1:
			agentInfo = [env.q, env.wall_setting, env.armLength]
		elif iAgent == 2:
			agentInfo = [iRobot, env.stiffHS, stiffReach]
			
		data = [record, parameters, agentInfo, meth]
		
		pickle.dump(data, output)
		output.close()
		
if iAgent == 2:
	env.body.relax()
