#!/usr/bin/env python

import rospy
import random
from read_config import read_config

map_y = (read_config()["map_size"][0])
map_x = (read_config()["map_size"][1])
move_list = (read_config()["move_list"])
start = (read_config()["start"])
goal = (read_config()["goal"])
walls = (read_config()["walls"])
pits = (read_config()["pits"])
move_list = (read_config()["move_list"])
max_iterations = (read_config()["max_iterations"])
threshold_difference = (read_config()["threshold_difference"])
reward_step = (read_config()["reward_for_each_step"])
reward_wall = (read_config()["reward_for_hitting_wall"])
reward_goal = (read_config()["reward_for_reaching_goal"])
reward_pit = (read_config()["reward_for_falling_in_pit"])

discount = (read_config()["discount_factor"])
prob_forward = (read_config()["prob_move_forward"])
prob_back = (read_config()["prob_move_backward"])
prob_left = (read_config()["prob_move_left"])
prob_right = (read_config()["prob_move_right"])

    

def getMdp(mapV):   
    tmapV =[[[0,0,0,0] for x in range(map_x)] for y in range(map_y)]
    tmapP =[["" for x in range(map_x)] for y in range(map_y)]
    tmapV[goal[0]][goal[1]] = [reward_goal,reward_goal,reward_goal,reward_goal]
    tmapP[goal[0]][goal[1]] = "GOAL"
    for pit in pits:
	tmapV[pit[0]][pit[1]] = [reward_pit,reward_pit,reward_pit,reward_pit]
	tmapP[pit[0]][pit[1]] = "PIT"
    for wall in walls:
	tmapP[wall[0]][wall[1]] = "WALL"

    for x in range(map_y):
	for y in range(map_x):
	    if([x,y] != goal and [x,y] not in walls and [x,y] not in pits):
		best = -100000
		policy = ""
		if([0,1] in move_list):
		    total = 0
		    forward = 0
		    backward = 0
		    left = 0
		    right = 0
		    mov = random.random()
		    if(mov < prob_forward):
		    	if(y+1>=map_x or [x,y+1] in walls):
				forward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				forward = (reward_step+discount*max(mapV[x][y+1][0],mapV[x][y+1][1],mapV[x][y+1][2],mapV[x][y+1][3]))
			tmapV[x][y][0] = 0.8*mapV[x][y][0]+0.2*forward
			if(tmapV[x][y][0] >= best):
				best = tmapV[x][y][0]
				policy = "E"
		    elif(mov < prob_forward+prob_back):
		    	if(y-1<0 or [x,y-1] in walls):
				backward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				backward = (reward_step+discount*max(mapV[x][y-1][0],mapV[x][y-1][1],mapV[x][y-1][2],mapV[x][y-1][3]))
			tmapV[x][y][0] = 0.8*mapV[x][y][0]+0.2*backward
			if(tmapV[x][y][0] >= best):
				best = tmapV[x][y][0]
				policy = "E"
		    elif(mov < prob_forward+prob_back+prob_right):
		    	if(x+1>=map_y or [x+1,y] in walls):
				right = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				right = (reward_step+discount*max(mapV[x+1][y][0],mapV[x+1][y][1],mapV[x+1][y][2],mapV[x+1][y][3]))
			tmapV[x][y][0] = 0.8*mapV[x][y][0]+0.2*right
			if(tmapV[x][y][0] >= best):
				best = tmapV[x][y][0]
				policy = "E"
		    else:
		    	if(x-1<0 or [x-1,y] in walls):
				left = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				left = (reward_step+discount*max(mapV[x-1][y][0],mapV[x-1][y][1],mapV[x-1][y][2],mapV[x-1][y][3]))
			tmapV[x][y][0] = 0.8*mapV[x][y][0]+0.2*left  
			if(tmapV[x][y][0] >= best):
				best = tmapV[x][y][0]
				policy = "E"
		if([0,-1] in move_list):
		    total = 0
		    forward = 0
		    backward = 0
		    left = 0
		    right = 0
		    mov = random.random()
		    if(mov < prob_back):
		    	if(y+1>=map_x or [x,y+1] in walls):
				forward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				forward = (reward_step+discount*max(mapV[x][y+1][0],mapV[x][y+1][1],mapV[x][y+1][2],mapV[x][y+1][3]))
			tmapV[x][y][1] = 0.8*mapV[x][y][1]+0.2*forward
			if(tmapV[x][y][1] >= best):
				best = tmapV[x][y][1]
				policy = "W"
		    elif(mov < prob_forward+prob_back):
		    	if(y-1<0 or [x,y-1] in walls):
				backward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				backward = (reward_step+discount*max(mapV[x][y-1][0],mapV[x][y-1][1],mapV[x][y-1][2],mapV[x][y-1][3]))
			tmapV[x][y][1] = 0.8*mapV[x][y][1]+0.2*backward
			if(tmapV[x][y][1] >= best):
				best = tmapV[x][y][1]
				policy = "W"
		    elif(mov < prob_forward+prob_back+prob_left):
		    	if(x+1>=map_y or [x+1,y] in walls):
				right = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				right = (reward_step+discount*max(mapV[x+1][y][0],mapV[x+1][y][1],mapV[x+1][y][2],mapV[x+1][y][3]))
			tmapV[x][y][1] = 0.8*mapV[x][y][1]+0.2*right
			if(tmapV[x][y][1] >= best):
				best = tmapV[x][y][1]
				policy = "W"
		    else:
		    	if(x-1<0 or [x-1,y] in walls):
				left = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				left = (reward_step+discount*max(mapV[x-1][y][0],mapV[x-1][y][1],mapV[x-1][y][2],mapV[x-1][y][3]))
			tmapV[x][y][1] = 0.8*mapV[x][y][1]+0.2*left  
			if(tmapV[x][y][0] >= best):
				best = tmapV[x][y][1]
				policy = "W"
		if([-1,0] in move_list):
		    total = 0
		    forward = 0
		    backward = 0
		    left = 0
		    right = 0
		    mov = random.random()
		    if(mov < prob_right):
		    	if(y+1>=map_x or [x,y+1] in walls):
				forward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				forward = (reward_step+discount*max(mapV[x][y+1][0],mapV[x][y+1][1],mapV[x][y+1][2],mapV[x][y+1][3]))
			tmapV[x][y][2] = 0.8*mapV[x][y][2]+0.2*forward
			if(tmapV[x][y][2] >= best):
				best = tmapV[x][y][2]
				policy = "N"
		    elif(mov < prob_right+prob_left):
		    	if(y-1<0 or [x,y-1] in walls):
				backward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				backward = (reward_step+discount*max(mapV[x][y-1][0],mapV[x][y-1][1],mapV[x][y-1][2],mapV[x][y-1][3]))
			tmapV[x][y][2] = 0.8*mapV[x][y][2]+0.2*backward
			if(tmapV[x][y][2] >= best):
				best = tmapV[x][y][2]
				policy = "N"
		    elif(mov < prob_back+prob_right+prob_left):
		    	if(x+1>=map_y or [x+1,y] in walls):
				right = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				right = (reward_step+discount*max(mapV[x+1][y][0],mapV[x+1][y][1],mapV[x+1][y][2],mapV[x+1][y][3]))
			tmapV[x][y][2] = 0.8*mapV[x][y][2]+0.2*right
			if(tmapV[x][y][2] >= best):
				best = tmapV[x][y][2]
				policy = "N"
		    else:
		    	if(x-1<0 or [x-1,y] in walls):
				left = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				left = (reward_step+discount*max(mapV[x-1][y][0],mapV[x-1][y][1],mapV[x-1][y][2],mapV[x-1][y][3]))
			tmapV[x][y][2] = 0.8*mapV[x][y][2]+0.2*left  
			if(tmapV[x][y][2] >= best):
				best = tmapV[x][y][2]
				policy = "N"	
		if([1,0] in move_list):
		    total = 0
		    forward = 0
		    backward = 0
		    left = 0
		    right = 0
		    mov = random.random()
		    if(mov < prob_left):
		    	if(y+1>=map_x or [x,y+1] in walls):
				forward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				forward = (reward_step+discount*max(mapV[x][y+1][0],mapV[x][y+1][1],mapV[x][y+1][2],mapV[x][y+1][3]))
			tmapV[x][y][3] = 0.8*mapV[x][y][3]+0.2*forward
			if(tmapV[x][y][3] >= best):
				best = tmapV[x][y][3]
				policy = "S"
		    elif(mov < prob_right+prob_left):
		    	if(y-1<0 or [x,y-1] in walls):
				backward = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				backward = (reward_step+discount*max(mapV[x][y-1][0],mapV[x][y-1][1],mapV[x][y-1][2],mapV[x][y-1][3]))
			tmapV[x][y][3] = 0.8*mapV[x][y][3]+0.2*backward
			if(tmapV[x][y][3] >= best):
				best = tmapV[x][y][3]
				policy = "S"
		    elif(mov < prob_forward+prob_right+prob_left):
		    	if(x+1>=map_y or [x+1,y] in walls):
				right = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				right = (reward_step+discount*max(mapV[x+1][y][0],mapV[x+1][y][1],mapV[x+1][y][2],mapV[x+1][y][3]))
			tmapV[x][y][3] = 0.8*mapV[x][y][3]+0.2*right
			if(tmapV[x][y][3] >= best):
				best = tmapV[x][y][3]
				policy = "S"
		    else:
		    	if(x-1<0 or [x-1,y] in walls):
				left = (reward_wall+discount*max(mapV[x][y][0],mapV[x][y][1],mapV[x][y][2],mapV[x][y][3]))
		    	else:
				left = (reward_step+discount*max(mapV[x-1][y][0],mapV[x-1][y][1],mapV[x-1][y][2],mapV[x-1][y][3]))
			tmapV[x][y][3] = 0.8*mapV[x][y][3]+0.2*left  
			if(tmapV[x][y][3] >= best):
				best = tmapV[x][y][3]
				policy = "S"
		tmapP[x][y] = policy
    return tmapV,tmapP

			
				
		     
