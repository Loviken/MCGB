class Record:
	def __init__(self):
		# Save motions like this?? -> [iter, activity, [n,s], [nG,sG]]
		# Activities:	0 - move in graph
		#				1 - reach
		#				2 - recover
		
		self.motions 	 = []
		
		
		# Save home state legend like this?? -> [iter, s, motivation]
		# Iter:	Iteration of creation
		# Motivations:	0 - new region
		#				1 - can't recover
		#				2 - region earlier discovered but now unreachable (only if purgeUnreach?)
		
		self.home_states = []	# [iter, s, motive]
		
		
		# Save MDP items like this?? -> [iter, P, R, V, Q]
		
		self.MDP = []
		
		# Can datasets be extracted from somewhere?
		
		
		
