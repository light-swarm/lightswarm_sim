from boid import Boid
import copy # for deep copying boids
import random

DISTANCE_BASED_NEIGHBORHOOD = True
K_NEAREST = 3
DISTANCE_SQUARED = 400

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
			self.boids.append(Boid(x,y))

	def get_random_boid_params(self):
		(minx, miny, maxx, maxy) = self.perimeter.bounds
		x = random.randrange(minx, maxx)
		y = random.randrange(miny, maxy)
		return x,y

	def boids_in_neighborhood(self, boids, target_boid):
		t = target_boid
		distanced_boids = [((b.x - t.x)**2 + (b.y - t.y)**2, b) for b in boids if b != t]

		if DISTANCE_BASED_NEIGHBORHOOD:
			boids = [b for (distance, b) in distanced_boids if distance < DISTANCE_SQUARED]
		else:
			distanced_boids.sort()
			boids = [b for (distance, b) in distanced_boids[:K_NEAREST]]

		return boids

	def update(self):
		self.keep_boids_in_world()

		cloned_boids = copy.deepcopy(self.boids)
		for boid in self.boids:
			neighbor_boids = self.boids_in_neighborhood(cloned_boids, boid)
			boid.update(neighbor_boids)



	def keep_boids_in_world(self):
		# teleport boids for the time being. wrap around
		(minx, miny, maxx, maxy) = self.perimeter.bounds
		for boid in self.boids:
			if boid.x < minx:
				boid.x = maxx
			if boid.x > maxx:
				boid.x = minx
			if boid.y < miny:
				boid.y = maxy
			if boid.y > maxy:
				boid.y = miny



