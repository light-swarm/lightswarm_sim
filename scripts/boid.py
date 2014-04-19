import math
import random
import numpy as np

SEP_RATE = 0.3
ALIGN_RATE = 0.15
SHEEP_RATE = 0.15
SPEED = 1.0  # x the refresh rate for speed in cms/sec
FAKE_MAXZ = 20.0
FAKE_MINZ = -20.0
GOAL_RATE = 0.15

def _normalized(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v/norm

def _repulsion_factor(distance, effect_horizon):
    # not a great function. net repulsion goes
    # down as the distance gets closer
    if distance < effect_horizon:
        repulse = (effect_horizon - distance) / effect_horizon
        repulse = repulse**6

        repulse = repulse * -0.2

        return repulse
    return 0

class Boid(object):
    def __init__(self, x, y, perimeter):
        self.pos = np.asarray([float(x),float(y)])
        self.id = random.randint(0, 1000)
        self._trail = [self.pos]
        self.vel = np.asarray([random.random() - 0.5 for i in self.pos])

        self.perimeter = perimeter
        self.last_neighbors = None


    def get_xy(self):
        return self.pos[0], self.pos[1]

    def get_past_xy(self):
        old_pos = self._trail[-1]
        return old_pos[0], old_pos[1]

    def get_theta(self):
        delta_pos = self.pos - self._trail[-1]
        return 180 * math.atan2(delta_pos[1], delta_pos[0]) / math.pi

    def update(self, neighbors, obstacles, goals):
        self.last_neighbors = neighbors
        self.update_velocity_boid_rules(neighbors)
        self.update_velocity_goal(goals)
        self.update_velocity_obstacles(obstacles)
        self.update_velocity_perimeter()
        self.pos += self.vel

        self._trail.insert(0, self.pos.copy())
        if len(self._trail) > 7:
            self._trail.pop()

        self.respect_perimeter()

    def update_velocity_perimeter(self):
        self.vel += self.perimeter.get_repulsive_vel(self.get_xy())

    def update_velocity_goal(self, goals):
        if len(goals) == 0:
            return
        goal_vel = np.asarray([0.0, 0.0])
        for goal in goals:
            goal_vel += goal.get_attraction_vel(self)
        self.vel += GOAL_RATE * goal_vel
        self.vel = _normalized(self.vel)

    def update_velocity_obstacles(self, obstacles):
        for obstacle in obstacles:
            repulsion_vel = obstacle.get_repulsive_vel(self.get_xy())
            self.vel += 3*repulsion_vel # no normalization after. speed can increase by 1


    def update_velocity_boid_rules(self, neighbors):
        n_neighbors = len(neighbors)
        if n_neighbors == 0:
            return

        neighbors_pos = np.asarray([n.pos for n in neighbors])
        neighbors_vel = np.asarray([n.vel for n in neighbors])



        ### BOID RULE1: head towards center of birds
        neighbors_pos_mean = np.mean(neighbors_pos, axis=0)     
        cohesion_vel = _normalized(neighbors_pos_mean - self.pos)       

        ### BOID RULE2: align vectors
        neighbors_vel_mean = np.mean(neighbors_vel, axis=0)

        alignment_vel = _normalized(neighbors_vel_mean - self.vel)

        ### BOID RULE3: don't get too close to other boids
        neighbors_vectors = [neigh_pos - self.pos for neigh_pos in neighbors_pos]
        neighbors_distance = [np.linalg.norm(v) for v in neighbors_vectors]
        #neighbors_vectors[neighbors_distance == 0.0] = self._default_random_vel

        neighbors_repulsion = [ _normalized(v)*_repulsion_factor(d, 20) for (v,d) in zip(neighbors_vectors, neighbors_distance)]
        neighbors_repulsion_total = np.sum(neighbors_repulsion, axis=0)

        sepration_vel = _normalized(neighbors_repulsion_total)

        self.vel += ALIGN_RATE * alignment_vel + SHEEP_RATE * cohesion_vel + SEP_RATE * sepration_vel
        self.vel = _normalized(self.vel) * SPEED

    def respect_perimeter(self):
        (minx, miny, maxx, maxy) = self.perimeter.get_bounds()
        if self.pos[0] < minx:
            self.pos[0] = minx
            self.vel[0] *= -1
        if self.pos[0] > maxx:
            self.pos[0] = maxx
            self.vel[0] *= -1               
        if self.pos[1] < miny:
            self.pos[1] = miny
            self.vel[1] *= -1
        if self.pos[1] > maxy:
            self.pos[1] = maxy
            self.vel[1] *= -1

        if self.pos.shape[0] == 3:
            if self.pos[2] > FAKE_MAXZ:
                self.pos[2] = FAKE_MAXZ
                self.vel[2] *= -1           
            if self.pos[2] < FAKE_MINZ:
                self.pos[2] = FAKE_MINZ
                self.vel[2] *= -1
                self.vel[0] *= -1          

    











