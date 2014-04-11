from flock import Flock
from shapely.geometry import Polygon


class World(object):
	""" 
	contains flock, perimeter and obstacles.
	doesn't have ros stuff other serialization to 
	messages.

	"""
	def __init__(self, perimeter, num_boids=20):
		self.perimeter = perimeter
		self.minx, self.miny, self.maxx, self.maxy = self.perimeter.bounds
		self.num_boids = num_boids
		self.flock = Flock(num_boids, perimeter)

	def update(self):
		self.flock.update()




if __name__ == '__main__':
	square_perimeter = Polygon([[-100.0, -100.0], [-100.0, 100.0], [100.0, 100.0], [100.0, -100.0]])
	world = World(square_perimeter)
	world.update()


