from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
import numpy as np

NULL_VECTOR = np.asarray([0.0, 0.0])

def _normalized(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v/norm


class Perimeter(object):
	def __init__(self, perimeter_points, buffer_size=10):
		self.outer_polygon = Polygon(perimeter_points)
		self.buffer_size = buffer_size
		self.inner_polygon = self.outer_polygon.buffer(-1 * buffer_size)

	def get_bounds(self):
		return self.outer_polygon.bounds

	def get_repulsive_vel(self, xy):
		point = Point(xy)
		containment_field = self.inner_polygon

		if containment_field.contains(point):
			return NULL_VECTOR

		vector_from_center = np.asarray([point.x - containment_field.centroid.x, point.y - containment_field.centroid.y])
		vector_to_center = -1 * vector_from_center

		line_to_center = LineString(((containment_field.centroid.x, containment_field.centroid.y), (point.x, point.y)))
		line_center_field = containment_field.intersection(line_to_center)
		distance_to_field = line_to_center.length - line_center_field.length

		repulsion_scale = distance_to_field / self.buffer_size

		return _normalized(vector_to_center) * repulsion_scale

	def get_perimeter_coordinates(self):
		return self.outer_polygon.exterior.coords

	def get_field_coordinates(self):
		return self.inner_polygon.exterior.coords






