from world import World
import time
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
        self._start_time = time.time()
        self._update_count = 0
        self._display_font = pygame.font.SysFont('monospace', 15)

        self._show_connections = False
        self.calibration_points = ((0, 0), (100, 0), (100, 100), (0, 100), (0, -60), (60, -60))

    def world_to_pixel(self, x, y):
        x = self.window_width * (x - self.minx) / (self.maxx - self.minx)
        y = self.window_height * (y - self.miny) / (self.maxy - self.miny)
        return int(x), int(y)

    def pixel_to_world(self, x, y):
        wx = float(x) * ((self.maxx - self.minx) / self.window_width) + self.minx
        wy = float(y) * ((self.maxy - self.miny) / self.window_height) + self.miny
        return wx, wy

    def render_world(self):
        self.render_perimeter()  
        self.render_calibration_points()      
        if self._show_connections:
            for boid in self.flock.boids:
                self.render_boid_neighborhood(boid)
        for obstacle in self.dynamic_obstacles:
            self.render_obstacle(obstacle, (255, 0, 0))
        for obstacle in self.static_obstacles:
            self.render_obstacle(obstacle, (255, 120, 120))
        for goal in self.goals:
            self.render_goal(goal)
        for agent in self.agents:
            self.render_agent(agent)
        for boid in self.flock.boids:
            self.render_boid(boid)


    def render_calibration_points(self):
        for anchor in self.calibration_points:
            px, py = self.world_to_pixel(*anchor)
            pygame.draw.circle(self.screen, (0, 255, 0), (px, py), 2)

    def render_agent(self, agent):
        agent_x, agent_y = self.world_to_pixel(*agent.get_xy())
        color = (255, 190, 255)
        if agent.has_been_stationary():
            color = (190, 255, 80)

        for pos in agent.trail[:30]:
            pos_x, pos_y = self.world_to_pixel(pos[0], pos[1])
            pygame.draw.circle(self.screen, color, (pos_x, pos_y), 1)
        pygame.draw.circle(self.screen, color, (agent_x, agent_y), 5)
        agent_label = self._display_font.render('%s' % agent.id, 1, color)
        self.screen.blit(agent_label, (agent_x + 5, agent_y + 5))



    def render_perimeter(self):
        boundary = [self.world_to_pixel(*c) for c in self.perimeter.get_perimeter_coordinates()]
        field = [self.world_to_pixel(*c) for c in self.perimeter.get_field_coordinates()]        
        pygame.draw.polygon(self.screen, (255, 120, 120), boundary, 3)
        pygame.draw.polygon(self.screen, (255, 120, 120), field, 1)        

    def render_goal(self, goal):
        goal_x, goal_y = self.world_to_pixel(*goal.get_xy())
        if self._show_connections:
            for boid in goal.neighborhood:
                b_x, b_y = self.world_to_pixel(*boid.get_xy())
                pygame.draw.line(self.screen, (40, 40, 90), (goal_x, goal_y), (b_x, b_y), 1)
        pygame.draw.circle(self.screen, (80, 190, 255), (goal_x, goal_y), 5)


    def render_obstacle(self, obstacle, color):
        boundary = [self.world_to_pixel(*c) for c in obstacle.get_coordinates()]
        dilation_boundary = [self.world_to_pixel(*c) for c in obstacle.get_dilation_coordinates()]
        pygame.draw.polygon(self.screen, color, boundary, 3)
        pygame.draw.polygon(self.screen, color, dilation_boundary, 1)


    def render_boid_neighborhood(self, boid):
        x,y = self.world_to_pixel(*boid.get_xy())
        for n in boid.last_neighbors:
            (nx, ny) = self.world_to_pixel(*n.get_xy())
            pygame.draw.line(self.screen, (40, 40, 40), (x,y), (nx, ny))        

    def render_boid(self, boid):
        x,y = self.world_to_pixel(*boid.get_xy())
        line_x, line_y = self.world_to_pixel(*boid.get_past_xy())
        pygame.draw.line(self.screen, (255, 50, 100), (x,y), (line_x, line_y), 2)       
        pygame.draw.circle(self.screen, (255, 255, 255), (x,y), 4)
        if boid.likes_anyone():
            pygame.draw.circle(self.screen, (0, 0, 255), (x,y), 2) 


    def render_status(self):
        elapsed_time = time.time() - self._start_time
        update_hz = float(self._update_count) / elapsed_time
        update_label = self._display_font.render('%.2f'%update_hz, 1, (255, 255, 0))
        self.screen.blit(update_label, (10, 10))

    def update(self):
        super(GraphicalWorld, self).update()

        self.handle_events()

        self.screen.blit(self.background, (0,0))
        self.render_world()
        self.render_status()
        pygame.display.flip()
        self._update_count += 1

    def handle_events(self):
        ev = pygame.event.get()
        for event in ev:
            if event.type == pygame.MOUSEBUTTONUP:
                xy = pygame.mouse.get_pos()
                x, y = self.pixel_to_world(xy[0], xy[1])                
                self.mouse_clicked(x, y)
            if event.type == pygame.KEYDOWN:
                self.key_pressed(event.key)

    def key_pressed(self, key):
        if key == pygame.K_x:
            self.set_goals([])
        if key == pygame.K_n:
            self._show_connections = False if self._show_connections else True

    def mouse_clicked(self, x, y):
        print 'setting goals %s %s ' % (x, y)
        self.set_goals([[x, y]])




    def run(self):
        offset = 0
        while True:
            self.update()
            #offset = i % self.maxx
            #self.set_goals([[0,i % self.maxy]])                
            self.set_dynamic_obstacles([[[0 + offset,40], [10 + offset,40], [10 + offset,50]]])
            #sleep(0.03)

if __name__ == '__main__':
    trapezoidal_world = [[-100, -100], [100, -100], [70, 120], [-70,120]]

    square_perimeter = [[-100.0, -100.0], [-100.0, 100.0], [100.0, 100.0], [100.0, -100.0]]
    world = GraphicalWorld(trapezoidal_world, 500, 500)
    world.run()


