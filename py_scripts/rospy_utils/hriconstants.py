#!/usr/bin/env python
from agents.coordinates import Point
import configparser

config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()

ENV = config['DEPLOYMENT ENVIRONMENT']['ENV']

HRI_ROS_PCKG = "hri_scenarios"

if ENV=='X':
	VREP_X_OFFSET = 7.725
	VREP_Y_OFFSET = 11.4
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
else:
	VREP_X_OFFSET = 7.55
	VREP_Y_OFFSET = 7.12
	REAL_X_OFFSET = 0.4
	REAL_Y_OFFSET = 1.55
	VREP_RECH_STATION = Point(+16.40+VREP_X_OFFSET, -3.66+VREP_Y_OFFSET)

	P_1 = Point(0.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_2 = Point(0.0/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_3 = Point(0.0/100.0 + 0.00, 672.5/100.0 + 0.00)
	P_4 = Point(0.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_5 = Point(1352.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_6 = Point(1352.0/100.0 + 0.00, 425.0/100.0 + 0.00)
	P_7 = Point(1352.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_8 = Point(1550.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_9 = Point(1550.0/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_10 = Point(1550.0/100.0 + 0.00, 672.5/100.0 + 0.00)
	P_11 = Point(1550.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_12 = Point(1802.5/100.0 + 0.00, 0.0/100.0 + 0.00)
	P_13 = Point(1802.5/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_14 = Point(185.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_15 = Point(185.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_16 = Point(1945.0/100.0 + 0.00, 0.0/100.0 + 0.00)
	P_17 = Point(1945.0/100.0 + 0.00, 695.0/100.0 + 0.00)
	P_18 = Point(2670.0/100.0 + 0.00, 0.0/100.0 + 0.00)
	P_19 = Point(2670.0/100.0 + 0.00, 695.0/100.0 + 0.00)
	P_20 = Point(2800.0/100.0 + 0.00, 0.0/100.0 + 0.00)
	P_21 = Point(2800.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_22 = Point(2970.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_23 = Point(2970.0/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_24 = Point(2970.0/100.0 + 0.00, 672.5/100.0 + 0.00)
	P_25 = Point(2970.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_26 = Point(3155.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_27 = Point(3155.0/100.0 + 0.00, 425.0/100.0 + 0.00)
	P_28 = Point(3155.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_29 = Point(4322.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_30 = Point(4322.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_31 = Point(4512.5/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_32 = Point(4512.5/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_33 = Point(4512.5/100.0 + 0.00, 672.5/100.0 + 0.00)
	P_34 = Point(4512.5/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_35 = Point(185.0/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_36 = Point(1352.0/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_37 = Point(1352.0/100.0 + 0.00, 672.5/100.0 + 0.00)
	P_38 = Point(185.0/100.0 + 0.00, 672.5/100.0 + 0.00)
	P_39 = Point(1550.0/100.0 + 0.00, 425.0/100.0 + 0.00)
	P_40 = Point(1945.0/100.0 + 0.00, 425.0/100.0 + 0.00)
	P_41 = Point(2670.0/100.0 + 0.00, 425.0/100.0 + 0.00)
	P_42 = Point(2970.0/100.0 + 0.00, 425.0/100.0 + 0.00)
	P_43 = Point(4512.0/100.0 + 0.00, 110.0/100.0 + 0.00)
	P_44 = Point(4512.0/100.0 + 0.00, 850.0/100.0 + 0.00)
	P_45 = Point(3155.0/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_46 = Point(4322.0/100.0 + 0.00, 299.5/100.0 + 0.00)
	P_47 = Point(3155.0/100.0 + 0.00, 672.5/100.0 + 0.00)
	P_48 = Point(4322.0/100.0 + 0.00, 672.5/100.0 + 0.00)

	T_1 = Point(92.50/100.0, 163.5/100)
	T_2 = Point(92.50/100.0, 761.2/100)
	T_3 = Point(1452.0/100.0, 761.2/100)
	T_4 = Point(1452.0/100.0, 163.5/100)
	T_5 = Point(3062.5/100.0, 163.5/100)
	T_6 = Point(4417.0/100.0, 163.5/100)
	T_7 = Point(3062.5/100.0, 761.2/100)
	T_8 = Point(4417.0/100.0, 761.2/100)
	T_9 = Point(2307.5/100.0, 347.5/100)
	T_10 = Point(2307.5/100.0, 55.0/100)

	W_1 = [P_1, P_13]
	W_2 = [P_1, P_4]
	W_3 = [P_4, P_11]
	W_4 = [P_39, P_11]
	W_5 = [P_35, P_36]
	W_6 = [P_35, P_38]
	W_7 = [P_38, P_37]
	W_8 = [P_36, P_37]
	W_9 = [P_39, P_40]
	W_10 = [P_12, P_13]
	W_11 = [P_40, P_17]
	W_12 = [P_17, P_19]
	W_13 = [P_41, P_19]
	W_14 = [P_12, P_20]
	W_15 = [P_20, P_21]
	W_16 = [P_41, P_42]
	W_17 = [P_21, P_43]
	W_18 = [P_43, P_44]
	W_19 = [P_25, P_44]
	W_20 = [P_42, P_25]
	W_21 = [P_45, P_46]
	W_22 = [P_45, P_47]
	W_23 = [P_47, P_48]
	W_24 = [P_46, P_48]

	WALLS = [W_1, W_2, W_3, W_4, W_5, W_6, W_7, W_8, W_9, W_10, W_11, W_12, W_13, W_14, W_15, W_16, W_17, W_18, W_19, W_20, W_21, W_22, W_23, W_24]
	TURN_POINTS = [T_1, T_2, T_3, T_4, T_5, T_6, T_7, T_8, T_9, T_10]
	DOORS = []

VREP_CLIENT_ID = 0
