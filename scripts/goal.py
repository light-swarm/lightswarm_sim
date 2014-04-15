import numpy as np

zero_vel = np.asarray([0.0, 0.0])

def _normalized(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v/norm

class Goal(object):
    def __init__(self, x, y):
        self.anchor_pos = np.asarray([float(x),float(y)])
        self.pos = self.anchor_pos
        self.neighborhood = []

    def get_xy(self):
        return self.pos[0], self.pos[1]

    def _is_boid_attracted(self, boid):
        if boid.id % 10 < 3:
            return True
        return False

    def _is_boid_attracted_nearest(self, boid):
        return boid.id in self.neighborhood_ids

    def get_attraction_vel(self, boid):
        if not self._is_boid_attracted_nearest(boid):
            return zero_vel

        delta_to_goal = self.pos - boid.pos
        return _normalized(delta_to_goal)


    def set_neighborhood(self, boids):
        self.neighborhood = boids
        self.neighborhood_ids = [b.id for b in self.neighborhood]