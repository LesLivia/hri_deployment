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





