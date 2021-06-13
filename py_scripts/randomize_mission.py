#!/usr/bin/env python
import random
from agents.mission import *

HUMANS = 1
f = open('mission.txt', 'w')

lines = []
for i in range(HUMANS):
	new = None
	random.seed()
	if random.randint(0, 100)>=50:
		lines.append(str('LEADER\n'))
	else:	
		lines.append(str('FOLLOWER\n'))

f.writelines(lines)
f.close()

