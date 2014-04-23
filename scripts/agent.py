import numpy as np
MAX_TRAIL = 100
zero_vel = np.asarray([0.0, 0.0])
BRAINWASH_RADIUS = 50
max_vel = np.asarray([100.0, 100.0])
TIME_NEEDED_FOR_HAS_BEEN_STATIONARY = 30 # longest constant award
STATIONARY_ENOUGH_SPEED = 10 # speed that we essentially take as stationary

def _normalized(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v/norm

class Agent(object):
    def __init__(self, x, y, id):
        self.pos = np.asarray([float(x), float(y)])
        self.id = id
        self.trail = [self.pos]
        self.distance_deltas = []
        self.alive_time = 1
        self.is_stationary_trail = []

        # cached values updated every time tick
        self._has_been_stationary = False
        self._vel = None
        self._speed = None
        self._normalized_vel = None

    def get_xy(self):
        return self.pos[0], self.pos[1]

    def same_agent(self, other_agent):
        return self.id == other_agent.id

    def update_from_msg(self, x, y):
        self.pos = np.asarray([float(x), float(y)])
        self.alive_time += 1

    def update(self):
        # called before boid calls
        self.trail.insert(0, self.pos)

        self._vel = self.get_vel()
        self._speed = np.linalg.norm(self._vel)
        self._normalized_vel = self._vel / self._speed if self._speed > 0 else zero_vel

        self.is_stationary_trail.insert(0, self._speed < STATIONARY_ENOUGH_SPEED)

        if len(self.is_stationary_trail) > MAX_TRAIL:
            self.is_stationary_trail.pop()
        if len(self.trail) > MAX_TRAIL:
            self.trail.pop()

        self._has_been_stationary = self.has_been_stationary()




    def has_been_stationary(self):
        if len(self.is_stationary_trail) < TIME_NEEDED_FOR_HAS_BEEN_STATIONARY:
            return False
        return all(self.is_stationary_trail[:TIME_NEEDED_FOR_HAS_BEEN_STATIONARY])

    def is_stationary(self):
        vel = self.get_vel()
        if np.linalg.norm(vel) < STATIONARY_ENOUGH_SPEED:
            return True
        return False

    def brainwash_boid(self, boid, distance, normalized_delta_to_agent):
        if distance > BRAINWASH_RADIUS:
            return

        normalized_delta_to_boid = -1 * normalized_delta_to_agent
        cosine_distance = normalized_delta_to_boid.dot(self._normalized_vel)

        if cosine_distance < 0.5 and self._speed > STATIONARY_ENOUGH_SPEED:
            boid.like_me(self.id, -0.5)
        boid.like_me(self.id, 0.01)

    def _is_boid_influenced(self, boid):
        if boid.id % 10 < 3:
            return True
        return False

    def get_vel(self):
        if len(self.trail) < 10:
            return max_vel
        return self.pos - self.trail[9]

    def get_influence_vel(self, boid):
        delta_to_agent = self.pos - boid.pos

        distance_to_agent = np.linalg.norm(delta_to_agent)

        normalized_delta_to_agent = zero_vel
        if distance_to_agent != 0:
            normalized_delta_to_agent = delta_to_agent / distance_to_agent

        self.brainwash_boid(boid, distance_to_agent, normalized_delta_to_agent)

        pull_boid = False

        if boid.likes_me(self.id):
            pull_boid = True
        if self.is_stationary() and self._is_boid_influenced(boid):
            pull_boid = True

        if pull_boid:
            return normalized_delta_to_agent
        else:
            return zero_vel



