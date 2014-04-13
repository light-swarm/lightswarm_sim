from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
import numpy as np

def _normalized(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v/norm

class Obstacle(object):
	def __init__(self, exterior_points, buffer_size=30):
		""" exterior points = list of [x,y]
		"""
		self.polygon = Polygon(exterior_points)
		self.buffer_size = buffer_size
		self.dilated_polygon = self.polygon.buffer(buffer_size)

	def get_center_xy(self):
		return self.polygon.centroid.x, self.polygon.centroid.y

	def get_coordinates(self):
		return self.polygon.exterior.coords

	def get_dilation_coordinates(self):
		return self.dilated_polygon.exterior.coords

	def get_repulsive_vel(self, xy):
		""" 
		xy = [x, y]
		
		return repulsion vector pointing away from the centroid
		of the obstacle. vector is scaled from 0 to 1 for strength of
		repulsion.
		"""
		point = Point(xy)
		if not self.dilated_polygon.contains(point):
			return np.asarray([0.0,0.0])

		vector_from_center = np.asarray([point.x - self.polygon.centroid.x, point.y - self.polygon.centroid.y])
		repulsion = _normalized(vector_from_center)

		# okay, now how much?
		#
		#                \              .            x - point
		#                 |              .           o - center
		#   o ------------|------> x ====.           = - distance from force_field
		#                 |              .        ---> - line from center
        #                /              .         ---| - line to boundary
        #             boundary       force_field
        #
        #  we want repulsion to be proportional to distance from force_field
        #  0 < repulsion < 1
        #  0 at force_filed
        #  1 at boundary or closer to the center
        #

		line_from_center = LineString(((point.x, point.y), (self.polygon.centroid.x, self.polygon.centroid.y)))
		line_to_boundary = self.polygon.intersection(line_from_center)

		distance_to_boundary = line_from_center.length - line_to_boundary.length
		# note that if the point has penetrated the boundary, the two lengths would be equal

		# note this is approximate because penetration may not have been at normal angle
		distance_from_force_field = self.buffer_size - distance_to_boundary
		distance_from_force_field = max(distance_from_force_field, 0) # buffer size assumed normal penetration

		# alright. now I have a distance_from_force_field that should be between 0 and buffer_size
		# rescale to 0, 1.0
		repulsion_scale = distance_from_force_field / self.buffer_size

		return repulsion * repulsion_scale


def obstacle_from_point(xy, buffer_size=30):
	x, y = xy
	d = 5
	perimeter = [[x-d, y-d], [x+d, y-d], [x+d, y+d], [x-d,y+d]]
	return Obstacle(perimeter, buffer_size)

if __name__ == '__main__':
	o = Obstacle(((0,0), (10,0), (10,10), (0,10)))
	p = (17,17)
	print o.get_repulsive_vel(p)


