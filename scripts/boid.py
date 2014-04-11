import math
import random
import numpy as np

SEP_RATE = 0.05
ALIGN_RATE = 0.1
SHEEP_RATE = 0.02
SPEED = 1.0  # x the refresh rate for speed in cms/sec

def _normalized(v):
	norm = np.linalg.norm(v)
	return v if norm == 0 else v/norm

def _repulsion_factor(distance):
	# not a great function. net repulsion goes
	# down as the distance gets closer
	if distance < 10:
		return -0.3
	return 0

class Boid(object):
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.heading = random.randrange(300) # in degrees
		self.last_neighbors = []
		self.vel_x = random.random() - 0.5
		self.vel_y = random.random() - 0.5




	def update(self, neighbors):
		self.last_neighbors = neighbors
		self.update_velocities(neighbors)
		self.x += self.vel_x
		self.y += self.vel_y


	def update_velocities(self, neighbors):
		n_neighbors = len(neighbors)
		if n_neighbors == 0:
			return

		pos = np.asarray([self.x, self.y])
		vel = np.asarray([self.vel_x, self.vel_y])
		neighbors_pos = np.asarray([(n.x, n.y) for n in neighbors])
		neighbors_vel = np.asarray([(n.vel_x, n.vel_y) for n in neighbors])



		### BOID RULE1: head towards center of birds
		neighbors_pos_mean = np.mean(neighbors_pos, axis=0)		
		cohesion_vel = _normalized(neighbors_pos_mean - pos)		

		### BOID RULE2: align vectors
		neighbors_vel_mean = np.mean(neighbors_vel, axis=0)

		alignment_vel = _normalized(neighbors_vel_mean - vel)

		### BOID RULE3: don't get too close to other boids
		neighbors_vectors = [neigh_pos - pos for neigh_pos in neighbors_pos]
		neighbors_distance = [np.linalg.norm(v) for v in neighbors_vectors]
		neighbors_repulsion = [ v*_repulsion_factor(d) for (v,d) in zip(neighbors_vectors, neighbors_distance)]
		neighbors_repulsion_total = np.sum(neighbors_repulsion, axis=0)

		sepration_vel = _normalized(neighbors_repulsion_total)

		vel = vel + ALIGN_RATE * alignment_vel + SHEEP_RATE * cohesion_vel + SEP_RATE * sepration_vel
		vel = _normalized(vel) * SPEED
		self.vel_x = vel[0]
		self.vel_y = vel[1]











