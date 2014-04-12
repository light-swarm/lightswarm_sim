from boid import Boid
import copy # for deep copying boids
import random
import numpy as np

DISTANCE_BASED_NEIGHBORHOOD = True
K_NEAREST = 3
DISTANCE = 30

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
		(minx, miny, maxx, maxy) = self.perimeter.bounds
		x = random.randrange(minx, maxx)
		y = random.randrange(miny, maxy)
		return x,y

	def boids_in_neighborhood(self, boids, target_boid):
		t = target_boid
		distanced_boids = [(np.linalg.norm(b.pos - t.pos),  b) for b in boids if b != t]

		if DISTANCE_BASED_NEIGHBORHOOD:
			boids = [b for (distance, b) in distanced_boids if distance < DISTANCE]
		else:
			distanced_boids.sort()
			boids = [b for (distance, b) in distanced_boids[:K_NEAREST]]

		return boids

	def update(self):
		obstacles = np.asarray([[0,0], [-100, -100], [100,-100], [-100, 100], [100,100]])
		cloned_boids = copy.deepcopy(self.boids)

		for boid in self.boids:
			obstacles[0][0] += 2 % 100
			neighbor_boids = self.boids_in_neighborhood(cloned_boids, boid)
			boid.update(neighbor_boids, obstacles)



