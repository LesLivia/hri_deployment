#!/usr/bin/env python
import os
import numpy
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import vrep_utils.vrep as vrep
from typing import List
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point

PI = 3.14

def init_walls():
	wall_pts = []
	density = 0.5
	for wall in const.WALLS:
		if wall[0].y == wall[1].y:
			for x in numpy.arange(wall[0].x, wall[1].x, density):
				wall_pts.append(Point(x, wall[0].y))
				# vrep.draw_point(const.VREP_CLIENT_ID, Point(x, wall[0].y))
		elif wall[0].x == wall[1].x:
			for y in numpy.arange(wall[0].y, wall[1].y, density):
				wall_pts.append(Point(wall[0].x, y))
				# vrep.draw_point(const.VREP_CLIENT_ID, Point(wall[0].x, y))
	# f = open("../scene_logs/walls.txt", "a")
	# for point in wall_pts:
    	#	 f.write("\n" + str(point))
	# f.close()
	return wall_pts

def close_to_wall(to_check: Point, walls: List[Point]):
	for point in walls:
		if to_check.distance_from(point) < 0.6:
			print(str(to_check.distance_from(point)))
			return True
	return False

def get_straight_line(start: Point, dest: Point):
	traj = []
	density = 1.0
	if abs(start.x-dest.x) > 2.0:
		m = (start.y - dest.y)/(start.x-dest.x)
		q = start.y - m*(start.x)
		if start.x >= dest.x:
			for x in numpy.arange(start.x, dest.x, -density):
				y = m*x + q
				traj.append(Point(x-const.VREP_X_OFFSET, y-const.VREP_Y_OFFSET))
				# vrep.draw_point(const.VREP_CLIENT_ID, Point(x, y))
		else:
			for x in numpy.arange(start.x, dest.x, +density):
				y = m*x + q
				traj.append(Point(x-const.VREP_X_OFFSET, y-const.VREP_Y_OFFSET))
				# vrep.draw_point(const.VREP_CLIENT_ID, Point(x, y))			
	else:
		if start.y >= dest.y:
			for y in numpy.arange(start.y, dest.y, -density):
				traj.append(Point(start.x-const.VREP_X_OFFSET, y-const.VREP_Y_OFFSET))
				# vrep.draw_point(const.VREP_CLIENT_ID, Point(start.x, y))
	
		else:		
			for y in numpy.arange(start.y, dest.y, +density):
				traj.append(Point(start.x-const.VREP_X_OFFSET, y-const.VREP_Y_OFFSET))	
				# vrep.draw_point(const.VREP_CLIENT_ID, Point(start.x, y))
	traj.append(Point(dest.x-const.VREP_X_OFFSET, dest.y-const.VREP_Y_OFFSET))
	# vrep.draw_point(const.VREP_CLIENT_ID, Point(dest.x, dest.y))		
	return traj
	
def plan_traj(start: Point, dest: Point, walls: List[Point]):
	crosses = False
	traj = []
	straight_line = get_straight_line(start, dest)
	for point in straight_line:
		crosses = close_to_wall(Point(point.x+const.VREP_X_OFFSET, point.y+const.VREP_Y_OFFSET), walls)
		if crosses:
			print('Straight line crosses wall')
			break

	if crosses:
		print('Calculate new trajectory')
		_curr = Point(start.x, start.y)
		_visited = []
		while crosses and len(_visited)<len(const.TURN_POINTS):
			_closest_turn = None
			_dist_to_closest_turn = 1000.0
			for point in const.TURN_POINTS:
				dist = _curr.distance_from(point)
				if dist < _dist_to_closest_turn and point not in _visited:
					_dist_to_closest_turn = dist
					_closest_turn = point
					_visited.append(point)
			new_segment = get_straight_line(_curr, _closest_turn)
			traj = traj + new_segment
			_curr = _closest_turn
			straight_line = get_straight_line(_curr, dest)
			for point in straight_line:
				crosses = close_to_wall(Point(point.x+const.VREP_X_OFFSET, point.y+const.VREP_Y_OFFSET), walls)
				if crosses:
					print('Straight line crosses wall')
					break

		new_segment = get_straight_line(_curr, dest)
		traj = traj + new_segment
	else:	
		traj = straight_line.copy()
	return traj

