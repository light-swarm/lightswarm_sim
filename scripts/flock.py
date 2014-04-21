from boid import Boid
import copy # for deep copying boids
import random
import numpy as np

DISTANCE_THRESHOLD = 15

class Flock(object):

	def __init__(self, num_boids, perimeter):
		self.num_boids = num_boids
		self.perimeter = perimeter
		self.boids = None
		self.setup_boids()

	def setup_boids(self):
		self.boids = []
		for i in range(self.num_boids):
			x,y = self.get_random_boid_params()
			self.boids.append(Boid(x,y, self.perimeter))

	def get_random_boid_params(self):
		(minx, miny, maxx, maxy) = self.perimeter.get_bounds()
		x = random.randrange(minx, maxx)
		y = random.randrange(miny, maxy)
		return x,y

	def boids_in_neighborhood(self, boids, target, use_distance=True, distance_threshold=DISTANCE_THRESHOLD, k_nearest=0):
		t = target # must have instance variable pos. ugh
		distanced_boids = [(np.linalg.norm(b.pos - t.pos),  b) for b in boids if b != t]

		if use_distance:
			boids = [b for (distance, b) in distanced_boids if distance < distance_threshold]
		else:
			distanced_boids.sort()
			boids = [b for (distance, b) in distanced_boids[:k_nearest]]

		return boids

	def update(self, obstacles, goals, agents):
		for boid in self.boids:
			boid.last_neighbors = None # hack to prevent recursion in deep copy below

		cloned_boids = copy.deepcopy(self.boids)

		for goal in goals:
			goal.set_neighborhood(self.boids_in_neighborhood(cloned_boids, goal, use_distance=False, k_nearest=7))


		for boid in self.boids:
			neighbor_boids = self.boids_in_neighborhood(cloned_boids, boid)
			boid.update(neighbor_boids, obstacles, goals, agents)



