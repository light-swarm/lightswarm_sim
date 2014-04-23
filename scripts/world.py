from flock import Flock
from shapely.geometry import Polygon
import numpy as np
from obstacle import Obstacle, obstacle_from_point
from goal import Goal
from perimeter import Perimeter

class World(object):
	""" 
	contains flock, perimeter and obstacles.
	doesn't have ros stuff other serialization to 
	messages.

	"""
	def __init__(self, perimeter_points, num_boids=30):
		self.perimeter = Perimeter(perimeter_points)
		self.minx, self.miny, self.maxx, self.maxy = self.perimeter.get_bounds()
		self.num_boids = num_boids
		self.flock = Flock(num_boids, self.perimeter)
		self.static_obstacles = []
		self.dynamic_obstacles = []
		self.goals = []
		self.agents = []
		self.set_static_obstacles()			

	def update(self):
		for agent in self.agents:
			agent.update()
		self.flock.update(self.static_obstacles + self.dynamic_obstacles, self.goals, self.agents)

	def set_dynamic_obstacles(self, polygon_perimeters):
		self.dynamic_obstacles = [Obstacle(p) for p in polygon_perimeters]

	def set_static_obstacles(self):
		corners = [[self.minx, self.miny], [self.minx, self.maxy], [self.maxx, self.maxy], [self.maxx, self.miny]]
		self.static_obstacles =  [obstacle_from_point(c) for c in corners]
	
		world_barrier_points = [[-65, 190], [49, 170], [-65, 150]]
		world_barrier = Obstacle(world_barrier_points)
		self.static_obstacles.append(world_barrier)

	def set_goals(self, xys):
		self.goals = [Goal(*xy) for xy in xys]

if __name__ == '__main__':
	square_perimeter = [[-100.0, -100.0], [-100.0, 100.0], [100.0, 100.0], [100.0, -100.0]]
	world = World(square_perimeter)
	world.update()


