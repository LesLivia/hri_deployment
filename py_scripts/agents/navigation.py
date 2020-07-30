#!/usr/bin/env python
import os
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import vrep_utils.vrep as vrep
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point

PI = 3.14

def is_in_rectangle(pos: Point, dest: Point, length: float, height: float):
	vrep.draw_point(const.VREP_CLIENT_ID, Point(dest.x-const.VREP_X_OFFSET, dest.y-const.VREP_Y_OFFSET))
	destXisIn = 0
	destYisIn = 0
	allowance = 0.95
	if abs(length) > abs(height):
		if length > 0:
			destXisIn = dest.x < pos.x + allowance*length and dest.x > pos.x - (1-allowance)*length
		else:
			destXisIn = dest.x < pos.x + (1-allowance)*length and dest.x > pos.x - allowance*length
		
		destYisIn = dest.y < pos.y + height/2 and dest.y > pos.y - height/2
	else:
		destXisIn = dest.x < pos.x + length/2 and dest.x > pos.x - length/2
		if height > 0:
			destYisIn = dest.y < pos.y + allowance*height and dest.y > pos.y - (1-allowance)*height
		else:
			destYisIn = dest.y < pos.y + (1-allowance)*height and dest.y > pos.y - allowance*height

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
	
	return [destIsOnTheLeft, destIsOnTheRight, destIsAhead, destIsBehind]





