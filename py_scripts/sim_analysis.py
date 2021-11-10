import scipy.stats
import math
import random
import sys
import configparser
import os

from agents.mobilerobot import MobileRobot
from agents.human import Human, follow_fatigue, follow_position
from agents.mission import *
from utils.logger import Logger
from typing import List, Tuple

config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()

LOGS_PATH = config['GENERAL SETTINGS']['LOGS_CP_PATH'].format(sys.argv[1])

LOGGER = Logger('SYM ANALYZER')

class Sim_Outcome:
    def __init__(self, served: List[Tuple], ftg: List[float], chg: List[float]):
        self.served = served
        self.scs = all([s[1] for s in self.served])
        self.fail = not self.scs
        self.ftg = ftg
        self.chg = chg


def clopper_pearson(x, n, alpha=0.05):
    """
    Estimate the confidence interval for a sampled Bernoulli random
    variable.
    `x` is the number of successes and `n` is the number trials (x <=
    n). `alpha` is the confidence level (i.e., the true probability is
    inside the confidence interval with probability 1-alpha). The
    function returns a `(low, high)` pair of numbers indicating the
    interval on the probability.
    """
    LOGGER.debug('Calculating confidence interval for {} successes out of {} trials...'.format(x, n))
    b = scipy.stats.beta.ppf
    lo = b(alpha / 2, x, n - x + 1)
    hi = b(1 - alpha / 2, x + 1, n - x)
    return 0.0 if math.isnan(lo) else lo, 1.0 if math.isnan(hi) else hi


def read_sims_from_logs(scenario_name: str, mis: Mission):
    sims = os.listdir(LOGS_PATH) 
    paths = [LOGS_PATH+s+'/' for s in sims if s.startswith('SIM')]
    outcomes = []
    for p in paths:
        served = []
        f = open(p + 'humansServed.log', 'r')
        lines = f.readlines()
        for i in range(len(mis.p)):
            served_line = list(filter(lambda l: 'human{}served'.format(i+1) in l, lines))
            if len(served_line)>0:
                ts = float(served_line[0].split(':')[0])
                served.append((ts, True))
            else:
                served.append((None, False))
        f.close()

        ftg = []
        f = open(p + 'humanFatigue.log', 'r')
        lines = f.readlines()
        lines = list(filter(lambda l: len(l)>1, lines))
        for i in range(len(mis.p)):
            lines_f = list(filter(lambda l: l.split(':')[1]=='hum{}'.format(i+1), lines))
            last_line = lines_f[-1]
            ftg.append(float(last_line.split(':')[2]))
        f.close()             

        chg = []
        f = open(p + 'robotBattery.log', 'r')
        lines = f.readlines()
        lines = list(filter(lambda l: len(l)>1, lines))
        last_line = lines[-1]
        chg.append(float(last_line.split(':')[1]))
        f.close()             

        outcomes.append(Sim_Outcome(served, ftg, chg))
    return outcomes

TAU = 100000
if len(sys.argv)>2:
    TAU = float(sys.argv[2])

outcomes = read_sims_from_logs(sys.argv[1], Mission([0]*1, [], []))
trials = len(outcomes)
successes = len(list(filter(lambda o: o.scs and o.served[-1][0]<=TAU, outcomes)))
lo, hi = clopper_pearson(successes, trials)
LOGGER.msg('Found {} successes over {} trials'.format(successes, trials))
LOGGER.msg('Confidence interval: ({:.3f}, {:.3f})'.format(lo, hi))
LOGGER.msg('Confidence interval width: {:.3f}'.format(hi-lo))

 