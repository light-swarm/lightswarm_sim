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
			self.render_boid(boid)

	def render_boid(self, boid):
		x,y = self.world_to_pixel(boid.x, boid.y)
		pygame.draw.circle(self.screen, (255, 255, 255), (x,y), 3)
		for n in boid.last_neighbors:
			(nx, ny) = self.world_to_pixel(n.x, n.y)
			pygame.draw.line(self.screen, (80, 80, 80), (x,y), (nx, ny))
		boid.last_neighbors = None

	def update(self):
		super(GraphicalWorld, self).update()
		self.screen.blit(self.background, (0,0))
		self.render_world()
		pygame.display.flip()


	def run(self):
		for i in range(1000):
			self.update()
			sleep(0.03)

if __name__ == '__main__':
	square_perimeter = Polygon([[-100.0, -100.0], [-100.0, 100.0], [100.0, 100.0], [100.0, -100.0]])
	world = GraphicalWorld(square_perimeter, 400, 400)
	world.run()


