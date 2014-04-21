import numpy as np
MAX_TRAIL = 100

class Agent(object):
	def __init__(self, x, y, id):
		self.pos = np.asarray([float(x), float(y)])
		self.id = id
		self.trail = [self.pos.copy()]
		self.alive_time = 1

	def get_xy(self):
		return self.pos[0], self.pos[1]

	def same_agent(self, other_agent):
		return self.id == other_agent.id

	def acquire_history(self, other_agent):
		assert self.id == other_agent.id
		self.trail.extend(other_agent.trail)
		self.trail = self.trail[:MAX_TRAIL]
		self.alive_time = other_agent.alive_time + 1


