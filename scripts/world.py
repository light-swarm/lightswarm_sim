from flock import Flock
from shapely.geometry import Polygon
import numpy as np
from obstacle import Obstacle, obstacle_from_point

class World(object):
	""" 
	contains flock, perimeter and obstacles.
	doesn't have ros stuff other serialization to 
	messages.

	"""
	def __init__(self, perimeter, num_boids=40):
		self.perimeter = perimeter
		self.minx, self.miny, self.maxx, self.maxy = self.perimeter.bounds
		self.num_boids = num_boids
		self.flock = Flock(num_boids, perimeter)
		self.static_obstacles = self.set_static_obstacles()
		self.dynamic_obstacles = []

	def update(self):
		self.flock.update(self.static_obstacles + self.dynamic_obstacles)

	def set_dynamic_obstacles(self, polygon_perimeters):
		self.dynamic_obstacles = [Obstacle(p) for p in polygon_perimeters]

	def set_static_obstacles(self):
		corners = [[self.minx, self.miny], [self.minx, self.maxy], [self.maxx, self.maxy], [self.maxx, self.miny]]
		return [obstacle_from_point(c) for c in corners]


if __name__ == '__main__':
	square_perimeter = Polygon([[-100.0, -100.0], [-100.0, 100.0], [100.0, 100.0], [100.0, -100.0]])
	world = World(square_perimeter)
	world.update()


