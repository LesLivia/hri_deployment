#!/usr/bin/env python
from agents.coordinates import Point

HRI_ROS_PCKG = "hri_scenarios"

VREP_X_OFFSET = 7.725
VREP_Y_OFFSET = 11.4

VREP_CLIENT_ID = 0

VREP_RECH_STATION = Point(+16.40+VREP_X_OFFSET, -3.66+VREP_Y_OFFSET)

# FLOOR LAYOUT
C_1_1 = Point(-7.7695+VREP_X_OFFSET, +1.25+VREP_Y_OFFSET)
C_1_2 = Point(+12.0805+VREP_X_OFFSET, +1.25+VREP_Y_OFFSET)
C_2_1 = Point(-7.7695+VREP_X_OFFSET, -3.425+VREP_Y_OFFSET)
C_2_2 = Point(+12.0805+VREP_X_OFFSET, -3.425+VREP_Y_OFFSET)
C_3_1 = Point(+12.0805+VREP_X_OFFSET, +9.35+VREP_Y_OFFSET)
C_3_2 = Point(+12.0805+VREP_X_OFFSET, -11.4+VREP_Y_OFFSET)
C_4_1 = Point(+26.6465+VREP_X_OFFSET, +9.35+VREP_Y_OFFSET)
C_4_2 = Point(+26.6465+VREP_X_OFFSET, -11.4+VREP_Y_OFFSET)
#
W_1_1 = Point(+16.9715+VREP_X_OFFSET, +6.31+VREP_Y_OFFSET)
W_DOOR1_B = Point(+16.9715+VREP_X_OFFSET, -0.1+VREP_Y_OFFSET)
W_DOOR1_A = Point(+16.9715+VREP_X_OFFSET, -2.1+VREP_Y_OFFSET)
W_1_4 = Point(+16.9715+VREP_X_OFFSET, -8.61+VREP_Y_OFFSET)
W_1_2 = Point(+23.0215+VREP_X_OFFSET, +6.31+VREP_Y_OFFSET)
W_1_3 = Point(+23.0215+VREP_X_OFFSET, -8.61+VREP_Y_OFFSET)
#
T_1 = Point(+14.52+VREP_X_OFFSET, -1.11+VREP_Y_OFFSET)
T_2 = Point(+14.60+VREP_X_OFFSET, +8.08+VREP_Y_OFFSET)
T_3 = Point(+14.52+VREP_X_OFFSET, -10.0+VREP_Y_OFFSET)
T_4 = Point(+25.0+VREP_X_OFFSET, +7.66+VREP_Y_OFFSET)
T_5 = Point(+25.0+VREP_X_OFFSET, -10.0+VREP_Y_OFFSET)
DOOR_1 = Point(+16.9715+VREP_X_OFFSET, -1+VREP_Y_OFFSET)
#
TURN_POINTS = [T_1]#, T_2, T_3, T_4, T_5]
DOORS = [DOOR_1]
#
WALL_1 = [C_1_1, C_1_2]
WALL_1_2 = [C_2_1, C_1_1]
WALL_2 = [C_2_1, C_2_2]
WALL_3_1 = [C_1_2, C_3_1]
WALL_3_2 = [C_3_2, C_2_2]
WALL_4 = [C_4_2, C_4_1]
WALL_3_4_1 = [C_3_1, C_4_1]
WALL_3_4_2 = [C_3_2, C_4_2]
#
WALL_W_1 = [W_1_1, W_1_2]
WALL_W_2 = [W_1_3, W_1_2]
WALL_W_3 = [W_1_4, W_1_3]
WALL_W_4_A = [W_1_4, W_DOOR1_A]
WALL_W_4_B = [W_DOOR1_B, W_1_1]
#
WALLS = [WALL_1, WALL_1_2, WALL_2, WALL_3_1, WALL_3_2, WALL_4, WALL_3_4_1, WALL_3_4_2,
		WALL_W_1, WALL_W_2, WALL_W_3, WALL_W_4_A, WALL_W_4_B] 

# HALL1_1 = Point(200.0, 400.00)
# HALL1_2 = Point(2200.00, 400.00)
# HALL1_3 = Point(2200.00, 600.00)
# HALL1_4 = Point(2400.00, 600.00)
# HALL2_1 = Point(200.0, 200.00)
# HALL2_2 = Point(2200.00, 200.00)
# HALL2_3 = Point(2200.00, 0.0)
# HALL2_4 = Point(2400.00, 0.0)

# WALL_1 = [HALL1_1, HALL1_2]
# WALL_1_2 = [HALL2_1, HALL1_1]
# WALL_2 = [HALL2_1, HALL2_2]
# WALL_3_1 = [HALL1_2, HALL1_3]
# WALL_3_2 = [HALL2_3, HALL2_2]
# WALL_4 = [HALL2_4, HALL1_4]
# WALL_3_4_1 = [HALL1_3, HALL1_4]
# WALL_3_4_2 = [HALL2_3, HALL2_4]

# WALLS = [WALL_1, WALL_1_2, WALL_2, WALL_3_1, WALL_3_2, WALL_4, WALL_3_4_1, WALL_3_4_2]






