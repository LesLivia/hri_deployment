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
		elif wall[0].x == wall[1].x:
			for y in numpy.arange(wall[0].y, wall[1].y, density):
				wall_pts.append(Point(wall[0].x, y))
	# f = open("../scene_logs/walls.txt", "a")
	# for point in wall_pts:
    	#	 f.write("\n" + str(point))
	# f.close()
	return wall_pts

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
				#vrep.draw_point(const.VREP_CLIENT_ID, Point(x, y))
		else:
			for x in numpy.arange(start.x, dest.x, +density):
				y = m*x + q
				traj.append(Point(x-const.VREP_X_OFFSET, y-const.VREP_Y_OFFSET))
				#vrep.draw_point(const.VREP_CLIENT_ID, Point(x, y))			
	else:
		if start.y >= dest.y:
			for y in numpy.arange(start.y, dest.y, -density):
				traj.append(Point(start.x-const.VREP_X_OFFSET, y-const.VREP_Y_OFFSET))
				#vrep.draw_point(const.VREP_CLIENT_ID, Point(start.x, y))
	
		else:		
			for y in numpy.arange(start.y, dest.y, +density):
				traj.append(Point(start.x-const.VREP_X_OFFSET, y-const.VREP_Y_OFFSET))	
				#vrep.draw_point(const.VREP_CLIENT_ID, Point(start.x, y))
	traj.append(Point(dest.x-const.VREP_X_OFFSET, dest.y-const.VREP_Y_OFFSET))
	#vrep.draw_point(const.VREP_CLIENT_ID, Point(dest.x, dest.y))		
	return traj
	
def plan_traj(start: Point, dest: Point, walls: List[Point]):
	crosses = False
	traj = []
	straight_line = get_straight_line(start, dest)
	for point in straight_line:
		for wall_point in walls:
			to_check = Point(point.x+const.VREP_X_OFFSET, point.y+const.VREP_Y_OFFSET)
			if to_check.distance_from(wall_point) < 0.5:
				print('Straight line crosses wall')
				crosses = True
				break
		if crosses:
			break

	if crosses:
		print('Calculate new trajectory')
		if dest.x > start.x:
			horz_line = get_straight_line(start, Point(dest.x, start.y))
			vert_line = get_straight_line(Point(dest.x, start.y), dest)
			traj = horz_line + vert_line
		else:
			vert_line = get_straight_line(start, Point(start.x, dest.y))
			horz_line = get_straight_line(Point(start.x, dest.y), dest)
			traj = vert_line + horz_line
	else:	
		traj = straight_line.copy()
	return traj
	

def is_in_rectangle(pos: Point, dest: Point, length: float, height: float):
	# visualization of dest in vrep
	vrep.draw_point(const.VREP_CLIENT_ID, Point(dest.x, dest.y))

	destXisIn = 0
	destYisIn = 0
	allowance = 0.95
	pts = []
	if abs(length) > abs(height):
		if length > 0:
			# visualization of projected rectangle in vrep
			pts = [Point(pos.x - (1-allowance)*length, pos.y + height/2), Point(pos.x + allowance*length, pos.y + height/2), Point(pos.x + allowance*length, pos.y - height/2), Point(pos.x - (1-allowance)*length, pos.y - height/2)]
		
			destXisIn = dest.x < pos.x + allowance*length and dest.x > pos.x - (1-allowance)*length
		else:
			# visualization of projected rectangle in vrep
			pts = [Point(pos.x + allowance*length, pos.y + height/2), Point(pos.x - (1-allowance)*length, pos.y + height/2), Point(pos.x - (1-allowance)*length, pos.y - height/2), Point(pos.x + allowance*length, pos.y - height/2)]

			destXisIn = dest.x < pos.x + (1-allowance)*length and dest.x > pos.x - allowance*length
		
		destYisIn = dest.y < pos.y + height/2 and dest.y > pos.y - height/2
	else:
		destXisIn = dest.x < pos.x + length/2 and dest.x > pos.x - length/2

		if height > 0:
			# visualization of projected rectangle in vrep
			pts = [Point(pos.x - length/2, pos.y - (1-allowance)*height), Point(pos.x + length/2, pos.y - (1-allowance)*height), Point(pos.x + length/2, pos.y + allowance*height), Point(pos.x - length/2, pos.y + allowance*height)]

			destYisIn = dest.y < pos.y + allowance*height and dest.y > pos.y - (1-allowance)*height
		else:
			# visualization of projected rectangle in vrep
			pts = [Point(pos.x + length/2, pos.y - (1-allowance)*height), Point(pos.x - length/2, pos.y - (1-allowance)*height), Point(pos.x - length/2, pos.y + allowance*height), Point(pos.x + length/2, pos.y + allowance*height)]

			destYisIn = dest.y < pos.y + (1-allowance)*-height and dest.y > pos.y - allowance*-height
	
	if destXisIn and destYisIn:
		vrep.draw_rectangle(const.VREP_CLIENT_ID, pts[0], pts[1], pts[2], pts[3])			
	return destXisIn and destYisIn


def get_dir_to_check(pos: Point, dest: Point, rob_theta: float, length: float, height: float):
	epsilon = 0.25
	destIsOnTheLeft = 0
	destIsOnTheRight = 0
	destIsAhead = 0
	destIsBehind = 0

	if abs(rob_theta-0) <= epsilon:
		destIsOnTheLeft = is_in_rectangle(pos, dest, height, length)		
		destIsOnTheRight = is_in_rectangle(pos, dest, height, -length)		
		destIsAhead = is_in_rectangle(pos, dest, length, height)		
		destIsBehind = is_in_rectangle(pos, dest, -length, height)
	elif abs(rob_theta-PI) <= epsilon:
		destIsOnTheLeft = is_in_rectangle(pos, dest, height, -length)
		destIsOnTheRight = is_in_rectangle(pos, dest, height, length)
		destIsAhead = is_in_rectangle(pos, dest, -length, height)
		destIsBehind = is_in_rectangle(pos, dest, length, height)
	elif abs(rob_theta-PI/2) <= epsilon:	        
		destIsOnTheLeft = is_in_rectangle(pos, dest, -length, height)
		destIsOnTheRight = is_in_rectangle(pos, dest, length, height)
		destIsAhead = is_in_rectangle(pos, dest, height, length)
		destIsBehind = is_in_rectangle(pos, dest, height, -length)
	elif abs(rob_theta-3/2*PI) <= epsilon:
		destIsOnTheLeft = is_in_rectangle(pos, dest, length, height)
		destIsOnTheRight = is_in_rectangle(pos, dest, -length, height)
		destIsAhead = is_in_rectangle(pos, dest, height, -length)
		destIsBehind = is_in_rectangle(pos, dest, height, length)
	
	return [destIsOnTheLeft, destIsOnTheRight, destIsAhead, destIsBehind]





