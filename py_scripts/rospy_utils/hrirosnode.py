#!/usr/bin/env python
import time
import subprocess
import rospy_utils.hriconstants as const
from typing import List

def rosrun_nodes(node: str, args: List[str]):
	command = "rosrun " + const.HRI_ROS_PCKG + " " + node 
	for arg in args:
		command += " " + arg
	# print(command)
	subprocess.Popen(command, shell=True)

def roskill_nodes(node: str):
	command = "rosnode kill " +  node
	subprocess.Popen(command, shell=True)

