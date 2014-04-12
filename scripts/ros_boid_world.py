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
        perimeter_polygons = []
        for polygon in obstacles.polygons:
            polygon_coords = []
            for point in polygon.points:
                polygon_coords.append([point.x, point.y])
            perimeter_polygons.append(polygon_coords)
        self.set_dynamic_obstacles(perimeter_polygons)


    def to_msg(self):
        world = World()
        boids = []
        for boid in self.flock.boids:
            ros_boid = Boid()
            x, y = boid.get_xy()
            ros_boid.location.x = x
            ros_boid.location.y = y
            ros_boid.theta = boid.get_theta()
            ros_boid.color = [255, 255, 255]
            boids.append(ros_boid)
        world.boids = boids

        for obstacle in self.dynamic_obstacles + self.static_obstacles:
            x, y = obstacle.get_center_xy()
            obstacle_boid = Boid()
            obstacle_boid.location.x = x
            obstacle_boid.location.y = y
            obstacle_boid.theta = 0
            obstacle_boid.color = [0, 0, 255]
            world.boids.append(obstacle_boid)

        # the main boid
        anchor_boid = Boid()
        anchor_boid.location.x = 70
        anchor_boid.location.y = 30
        anchor_boid.theta = 0
        anchor_boid.color = [0, 255, 0]
        world.boids.append(anchor_boid)
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

