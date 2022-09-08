#!/usr/bin/env python
import configparser
import numpy
import rospy_utils.hriconstants as const
import vrep_utils.vrep as vrep
from typing import List
from agents.coordinates import Point
from functools import cmp_to_key
from utils.logger import Logger

config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()

ENV = config['DEPLOYMENT ENVIRONMENT']['ENV']

LOGGER = Logger('NAVIGATION')

PI = 3.14


def init_walls(draw=False):
    wall_pts = []
    density = 0.5
    for wall in const.WALLS:
        if wall[0].y == wall[1].y:
            for x in numpy.arange(wall[0].x, wall[1].x, density):
                wall_pts.append(Point(x, wall[0].y))
                if draw:
                    LOGGER.debug('drawing point {} {}...'.format(x, wall[0].y))
                    vrep.draw_point(const.VREP_CLIENT_ID, Point(x, wall[0].y))
        elif wall[0].x == wall[1].x:
            for y in numpy.arange(wall[0].y, wall[1].y, density):
                wall_pts.append(Point(wall[0].x, y))
                if draw:
                    LOGGER.debug('drawing point {} {}...'.format(wall[0].x, y))
                    vrep.draw_point(const.VREP_CLIENT_ID, Point(wall[0].x, y))
    return wall_pts


def close_to_wall(to_check: Point, walls: List[Point]):
    for point in walls:
        if to_check.distance_from(point) < 0.2:
            return True
    return False


def get_straight_line(start: Point, dest: Point, density=1.5, draw=False):
    traj = []
    if abs(start.x - dest.x) > 2.0:
        m = (start.y - dest.y) / (start.x - dest.x)
        q = start.y - m * (start.x)
        if start.x >= dest.x:
            for x in numpy.arange(start.x, dest.x, -density):
                y = m * x + q
                traj.append(Point(x - const.VREP_X_OFFSET, y - const.VREP_Y_OFFSET))
                if draw:
                    vrep.draw_point(const.VREP_CLIENT_ID, Point(x, y))
        else:
            for x in numpy.arange(start.x, dest.x, +density):
                y = m * x + q
                traj.append(Point(x - const.VREP_X_OFFSET, y - const.VREP_Y_OFFSET))
                if draw:
                    vrep.draw_point(const.VREP_CLIENT_ID, Point(x, y))
    else:
        if start.y >= dest.y:
            for y in numpy.arange(start.y, dest.y, -density):
                traj.append(Point(start.x - const.VREP_X_OFFSET, y - const.VREP_Y_OFFSET))
                if draw:
                    vrep.draw_point(const.VREP_CLIENT_ID, Point(start.x, y))

        else:
            for y in numpy.arange(start.y, dest.y, +density):
                traj.append(Point(start.x - const.VREP_X_OFFSET, y - const.VREP_Y_OFFSET))
                if draw:
                    vrep.draw_point(const.VREP_CLIENT_ID, Point(start.x, y))
    traj.append(Point(dest.x - const.VREP_X_OFFSET, dest.y - const.VREP_Y_OFFSET))
    if draw:
        vrep.draw_point(const.VREP_CLIENT_ID, Point(dest.x, dest.y))
    return traj


def dist_cmp(item1, item2):
    if item1.distance_from(dest_pt) < item2.distance_from(dest_pt):
        return -1
    elif item1.distance_from(dest_pt) > item2.distance_from(dest_pt):
        return 1
    else:
        return 0


def plan_traj(start: Point, dest: Point, walls: List[Point], draw=False):
    crosses = False
    traj = []
    straight_line = get_straight_line(start, dest, 0.25, draw)
    for point in straight_line:
        crosses = close_to_wall(Point(point.x + const.VREP_X_OFFSET, point.y + const.VREP_Y_OFFSET), walls)
        if crosses:
            LOGGER.debug('Straight line crosses wall')
            break

    global dest_pt
    dest_pt = dest
    cmp_items_py3 = cmp_to_key(dist_cmp)
    turn_pts = const.TURN_POINTS.copy()
    if (24 < dest.x < 30 and 2.8 < dest.y < 17.7) or (24 < start.x < 30 and 2.8 < start.y < 17.7):
        turn_pts += const.DOORS.copy()
    turn_pts.sort(key=cmp_items_py3)
    if crosses:
        LOGGER.info('Calculating new trajectory...')
        _curr = Point(start.x, start.y)
        _visited = []
        while crosses and len(_visited) < len(turn_pts):
            _closest_turn = None
            _dist_to_closest_turn = 1000.0
            for point in turn_pts:
                dist = point.distance_from(dest)
                if dist < _dist_to_closest_turn and point not in _visited:
                    print('{} {}'.format(point.x, point.y))
                    straight_line = get_straight_line(_curr, point, 1.5, draw)
                    for point in straight_line:
                        crosses = close_to_wall(Point(point.x + const.VREP_X_OFFSET, point.y + const.VREP_Y_OFFSET),
                                                walls)
                        if crosses:
                            LOGGER.debug('Straight line crosses wall')
                            break
                    else:
                        _dist_to_closest_turn = dist
                        _closest_turn = point
            _visited.append(_closest_turn)
            new_segment = get_straight_line(_curr, _closest_turn, 1.5, draw)
            traj = traj + new_segment
            _curr = _closest_turn
            straight_line = get_straight_line(_curr, dest, 0.25, draw)
            for point in straight_line:
                crosses = close_to_wall(Point(point.x + const.VREP_X_OFFSET, point.y + const.VREP_Y_OFFSET), walls)
                if crosses:
                    LOGGER.debug('Straight line crosses wall')
                    break
        new_segment = get_straight_line(_curr, dest, 1.5, draw)
        traj = traj + new_segment
    else:
        straight_line = get_straight_line(start, dest, 1.5, draw)
        traj = straight_line.copy()
    return traj
