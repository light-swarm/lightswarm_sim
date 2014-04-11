#!/usr/bin/env python

import math
import rospy
import yaml

from lightswarm_core.msg import Shadow
from lightswarm_core.msg import Obstacles
from lightswarm_core.msg import World
from lightswarm_core.msg import Boid

from graphical_world import GraphicalWorld
from shapely.geometry import Polygon



CONFIG_FILE = 'lightswarm_core/params/config.yaml'

class RosBoidWorld(GraphicalWorld):

    def __init__(self):
        rospy.init_node('ros_boid_world')
        config_filename = rospy.get_param('config_file', CONFIG_FILE)
        self.config_map = yaml.load(open(config_filename))

        perimeter = self.config_map.get('sim_world_perimeter')
        width = self.config_map.get('sim_window_width')
        height = self.config_map.get('sim_window_height')


        super(RosBoidWorld, self).__init__(Polygon(perimeter), width, height)
        self.sub = rospy.Subscriber('/obstacles', Obstacles, self.obstacles_callback)
        self.pub = rospy.Publisher('/world', World)




    def obstacles_callback(self, obstacles):
        pass

    def to_msg(self):
        world = World()
        boids = []
        for boid in self.flock.boids:
            ros_boid = Boid()
            ros_boid.location.x = boid.x
            ros_boid.location.y = boid.y
            ros_boid.theta = 180 * math.atan2(boid.vel_y, boid.vel_x) / math.pi
            ros_boid.color = [255, 255, 255]
            boids.append(ros_boid)
        world.boids = boids
        return world

    def run(self):
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.update()
            self.pub.publish(self.to_msg())
            r.sleep()

if __name__ == '__main__':
    world = RosBoidWorld()
    world.run()

