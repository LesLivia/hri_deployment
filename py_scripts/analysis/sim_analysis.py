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
from typing import List

config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()

LOGS_PATH = config['GENERAL SETTINGS']['LOGS_CP_PATH'].format(sys.argv[1])

LOGGER = Logger('SimAnalysis')

class Sim_Outcome:
    def __init__(self, served: List[Bool], ftg: List[Double], chg: List[Double]):
        self.served = served
        self.scs = all(self.served)
        self.fail = not self.scs
        self.ftg = ftg
        self.chg = chg


def clopper_pearson(x, n, alpha=0.05):
    """Estimate the confidence interval for a sampled Bernoulli random
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
    paths = [LOGS_PATH+s+'/' for s in sims]
    outcomes = []
    for p in paths:
        served = []
        f = open(p + 'humansServed.log', 'r')
        lines = f.readlines()
        for i in range(len(mis.p)):
            if any([l.contains('human{}Served'.format(i+1) for l in lines)]):
                served.append(True)
            else:
                served.append(False)
        f.close()

        outcomes.append(Sim_Outcome(served, [], []))
    return outcomes

print(read_sims_from_logs(sys.argv[1]), Mission([0]*1, [], []))