#!/usr/bin/env python

import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
from typing import List
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point
from enum import Enum
from utils.logger import Logger

LOGGER = Logger('ALL HUMANS')

class FatigueProfile(Enum):
    YOUNG_HEALTHY = 1  # associate the different profiles to ints
    YOUNG_SICK = 2
    ELDERLY_HEALTHY = 3
    ELDERLY_SICK = 4

    # RETURN THE VELOCITY PROFILE DEPENDING ON THE FATIGUE PROFILE
    def get_def_rates(self):
        if self == FatigueProfile.YOUNG_HEALTHY:
            return [0.0004, 0.0005]
        elif self == FatigueProfile.YOUNG_SICK:
            return [0.004, 0.005]
        elif self == FatigueProfile.ELDERLY_HEALTHY:
            return [0.0005, 0.0004]
        elif self == FatigueProfile.ELDERLY_SICK:
            return [0.005, 0.004]


class Human:
    def __init__(self, hum_id, speed, ftg_profile, fw_profile):
        self.hum_id = hum_id
        self.speed = speed
        self.ftg_profile = ftg_profile
        self.fw_profile = fw_profile
        self.moving = False
        self.position = None
        self.fatigue = 0.0
        self.f_0 = 0
        self.last_switch = 0.0
        self.emg_walk = []
        self.emg_rest = []
        self.lambdas = [0.0005]
        self.mus = [0.0005]
        self.def_bursts_mov = []
        self.def_bursts_rest = []
        self.cand_bursts_mov = []
        self.cand_bursts_rest = []
        self.LOGGER = Logger('HUMAN {}'.format(hum_id))

    def set_position(self, position: Position):  # define set and get functions:
        self.position = position  # set -> associate the property to it's current value

    # get -> return the current value of the property
    def get_position(self):
        return self.position

    def set_fatigue(self, ftg: float):
        self.fatigue = ftg

    def get_fatigue(self):
        return self.fatigue

    def set_f_o(self, f_0: float):
        self.f_0 = f_0

    def get_f_o(self):
        return self.f_0

    def set_last_switch(self, t: float):
        self.last_switch = t

    def get_last_switch(self):
        return self.last_switch

    def set_emg_signal(self, state: str, emg: List[float]):
        if state == 'm':
            self.emg_walk += emg
        else:
            self.emg_rest += emg

    def get_emg_signal(self, state: str):
        return self.emg_walk if state == 'm' else self.emg_rest

    def set_lambdas(self, l: float):
        self.lambdas.append(l)

    def get_lambdas(self):
        return self.lambdas

    def set_mus(self, m: float):
        self.mus.append(m)

    def get_mus(self):
        return self.mus

    def set_sim_running(self, run):  # define the flag if it's running
        self.sim_running = run

    def is_sim_running(self):  # is it running?
        return self.sim_running

    def set_is_moving(self, moving: bool):
        self.moving = moving

    def is_moving(self):
        return self.moving


# READ DATA FROM ALL THE HUMANS
def start_reading_data(humans: List[Human]):

    for hum in humans:
        f = open('../scene_logs/emg_to_ftg{}.log'.format(hum.hum_id), 'r+')
        f.truncate(0)
        f.close()

    node = 'humSensorsSub.py'

    LOGGER.info('Subscribing to position data...')
    pool = Pool()
    pool.starmap(hriros.rosrun_nodes, [(node, '')])

    node = 'humFtgSub.py'

    LOGGER.info('Subscribing to fatigue data...')
    pool = Pool()
    pool.starmap(hriros.rosrun_nodes, [(node, '')])

    node = 'humServiceSub.py'

    pool = Pool()
    LOGGER.info('Subscribing to served data...')
    pool.starmap(hriros.rosrun_nodes, [(node, '')])

    for hum in humans:
        hum.set_sim_running(1)  # set humans to 'running'


# TRACK THE POSITION OF EACH HUMAN
def follow_position(hums: List[Human]):
    filename = '../scene_logs/humanPosition.log'
    f = open(filename, 'r')
    lines = f.read().splitlines()
    for hum in hums:
        hum_lines = list(filter(lambda l: len(l) > 1 and l.split(':')[1] == 'hum{}'.format(hum.hum_id), lines))
        if len(hum_lines) > 0:
            n_humans = 1
            to_read = hum_lines[hum.hum_id - n_humans]
            # humId = int((line.split(':')[1]).replace('hum', ''))
            # hum = hums[humId-1]

            new_position = Position.parse_position(to_read.split(':')[2])
            new_position.x += const.VREP_X_OFFSET
            new_position.y += const.VREP_Y_OFFSET
        else:
            new_position = None

        hum.LOGGER.debug('Updating position to ({:.2f}, {:.2f})'.format(new_position.x, new_position.y))
        hum.set_position(new_position)


# TRACK THE FATIGUE OF EACH HUMAN
def follow_fatigue(hums: List[Human]):
    filename = '../scene_logs/humanFatigue.log'
    f = open(filename, 'r')
    lines = f.read().splitlines()
    for hum in hums:
        hum_lines = lines
        n_humans = 1
        if len(hum_lines) > 0:
            to_read = hum_lines[hum.hum_id - n_humans]
            # humId = int((line.split(':')[1]).replace('hum', ''))
            # hum = hums[humId-1]
            new_ftg = float((to_read.split(':')[2]))
        else:
            new_ftg = None

        hum.LOGGER.debug('Updating fatigue to ({:.2f})'.format(new_ftg))
        hum.set_fatigue(new_ftg)
