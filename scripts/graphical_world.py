from world import World
from time import sleep
from shapely.geometry import Polygon
import pygame

class GraphicalWorld(World):

	def __init__(self, perimeter, window_width, window_height):
		super(GraphicalWorld, self).__init__(perimeter)

		self.window_width = window_width
		self.window_height = window_height
		pygame.init()
		self.screen = pygame.display.set_mode((window_width, window_height))

		self.background = pygame.Surface(self.screen.get_size()).convert()
		self.background.fill((20, 20, 20))

	def world_to_pixel(self, x, y):
		x = self.window_width * (x - self.minx) / (self.maxx - self.minx)
		y = self.window_height * (y - self.miny) / (self.maxy - self.miny)
		return int(x), int(y)

	def render_world(self):
		for boid in self.flock.boids:
			self.render_boid_neighborhood(boid)
		for boid in self.flock.boids:
			self.render_boid(boid)
		for obstacle in self.dynamic_obstacles:
			self.render_obstacle(obstacle, (255, 0, 0))
		for obstacle in self.static_obstacles:
			self.render_obstacle(obstacle, (255, 120, 120))


	def render_obstacle(self, obstacle, color):
		boundary = [self.world_to_pixel(*c) for c in obstacle.get_coordinates()]
		dilation_boundary = [self.world_to_pixel(*c) for c in obstacle.get_dilation_coordinates()]
		pygame.draw.polygon(self.screen, color, boundary, 3)
		pygame.draw.polygon(self.screen, color, dilation_boundary, 1)


	def render_boid_neighborhood(self, boid):
		x,y = self.world_to_pixel(*boid.get_xy())
		for n in boid.last_neighbors:
			(nx, ny) = self.world_to_pixel(*n.get_xy())
			pygame.draw.line(self.screen, (60, 60, 60), (x,y), (nx, ny))		

	def render_boid(self, boid):
		x,y = self.world_to_pixel(*boid.get_xy())
		line_x, line_y = self.world_to_pixel(*boid.get_past_xy())
		pygame.draw.circle(self.screen, (255, 255, 255), (x,y), 4)
		pygame.draw.line(self.screen, (255, 50, 100), (x,y), (line_x, line_y), 2)
		boid.last_neighbors = None  ### awful hack

	def update(self):
		super(GraphicalWorld, self).update()
		self.screen.blit(self.background, (0,0))
		self.render_world()
		pygame.display.flip()


	def run(self):
		for i in range(2000):
			self.update()
			offset = i % self.maxx
			self.set_dynamic_obstacles([[[0 + offset,0], [10 + offset,0], [10 + offset,10]]])
			#sleep(0.03)

if __name__ == '__main__':


	square_perimeter = Polygon([[-100.0, -100.0], [-100.0, 100.0], [100.0, 100.0], [100.0, -100.0]])
	world = GraphicalWorld(square_perimeter, 400, 400)
	world.run()


