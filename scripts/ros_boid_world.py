#!/usr/bin/env python

import math
import rospy
import yaml

from lightswarm_core.msg import Shadow
from lightswarm_core.msg import Obstacles
from lightswarm_core.msg import World
from lightswarm_core.msg import Boid
from lightswarm_core.msg import Agents

from graphical_world import GraphicalWorld
from shapely.geometry import Polygon

from agent import Agent


CONFIG_FILE = 'lightswarm_core/params/config.yaml'

class RosBoidWorld(GraphicalWorld):

    def __init__(self):
        rospy.init_node('ros_boid_world')
        config_filename = rospy.get_param('config_file', CONFIG_FILE)
        self.config_map = yaml.load(open(config_filename))

        perimeter = self.config_map.get('sim_world_perimeter')
        width = self.config_map.get('sim_window_width')
        height = self.config_map.get('sim_window_height')


        super(RosBoidWorld, self).__init__(perimeter, width, height)
        self.sub = rospy.Subscriber('/obstacles', Obstacles, self.obstacles_callback)
        self.agents_sub = rospy.Subscriber('/agents', Agents, self.agents_callback)
        self.pub = rospy.Publisher('/world', World)



    def obstacles_callback(self, obstacles):
        perimeter_polygons = []
        for polygon in obstacles.polygons:
            polygon_coords = []
            for point in polygon.points:
                polygon_coords.append([point.x, point.y])
            perimeter_polygons.append(polygon_coords)
        self.set_dynamic_obstacles(perimeter_polygons)

    def agents_callback(self, agents):
        new_agent_list = []
        existing_agent_map = {a.id : a for a in self.agents}
        for agent_msg in agents.agents:
            if agent_msg.id in existing_agent_map:
                agent = existing_agent_map[agent_msg.id]
                agent.update_from_msg(agent_msg.location.x, agent_msg.location.y)
            else:
                agent = Agent(agent_msg.location.x, agent_msg.location.y, agent_msg.id)
            new_agent_list.append(agent)
        # all agents not getting an update die... slowly... in the rain
        self.agents = new_agent_list






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
        for anchor in self.calibration_points:
            anchor_boid = Boid()
            anchor_boid.location.x = anchor[0]
            anchor_boid.location.y = anchor[1]
            anchor_boid.theta = 0
            anchor_boid.color = [0, 255, 0]
            world.boids.append(anchor_boid)

        for goal in self.goals:
            x, y = goal.get_xy()
            goal_boid = Boid()
            goal_boid.location.x = x
            goal_boid.location.y = y
            goal_boid.theta = 0
            goal_boid.color = [255, 0, 0]
            world.boids.append(goal_boid)            

        return world

    def run(self):
        r = rospy.Rate(18)

        while not rospy.is_shutdown():
            self.update()
            self.pub.publish(self.to_msg())
            r.sleep()

if __name__ == '__main__':
    world = RosBoidWorld()
    world.run()

