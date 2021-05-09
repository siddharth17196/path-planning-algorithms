#!/usr/bin/env python
# coding: utf-8

import matplotlib.pyplot as plt
import numpy as np
import argparse


class Node:
	def __init__(self, x, y):
		self.x = x
		self.y = y


def attractive_potential(x, y, goal, K_att, func):
	if func == 'p':
		return K_att * (np.hypot(x - goal.x, y - goal.y))**2
	else:
		return K_att * (np.hypot(x - goal.x, y - goal.y))


def repulsive_potential(x, y, K_rep):
	global obslist
	influence_region = 1
	min_dist = float("inf")
	dist_list = [np.hypot(x-o[0], y-o[1]) for o in obslist]
	closest_index = dist_list.index(min(dist_list))
	dq = np.hypot(x - obslist[closest_index][0], y - obslist[closest_index][1])
	region = obslist[closest_index][2] + influence_region
	if dq <= region:
		if dq <= 0.3:
			dq = 0.3
		return 0.5 * K_rep * (1/dq - 1/region) ** 2 # gamma = 2
	else:
		return 0

def get_potential_matrix(goal, grid_size, params, func, minx=0, maxx=30):
	global obslist
	min_x = min_y = minx
	max_x = max_y = maxx
	x_grid = int(round((max_x - min_x) / grid_size))
	y_grid = int(round((max_y - min_y) / grid_size))

	potential_grid = [[0.0 for i in range(y_grid)] for i in range(x_grid)]

	for i in range(x_grid):
		x = i * grid_size + min_x
		for j in range(y_grid):
			y = j * grid_size + min_y
			u_att = attractive_potential(x, y, goal, params[0], func)
			u_rep = repulsive_potential(x, y, params[1])
			uf = u_rep + u_att
			potential_grid[i][j] = uf

	return potential_grid, min_x, min_y


def potential_field_planning(start, goal, grid_size, params, func):
	pmap, minx, miny = get_potential_matrix(goal, grid_size, params, func)
	robot_path = Node(start.x, start.y)
	move = []
	directions = [0,1,1]	# move at max 1 step
	for i in directions:
		for j in directions:
			move.append([i,j])
	move.remove([0,0])	# [0,0] => robot doesnt move
	path = [start]
	d = np.hypot(start.x - goal.x, start.y - goal.y)
	lim = 0
	while d >= grid_size:
		min_potential = float("inf")
		min_pot_x, min_pot_y = -100, -100
		for i, _ in enumerate(move):
			moved_x = int(robot_path.x + move[i][0])
			moved_y = int(robot_path.y + move[i][1])
			if moved_x >= len(pmap) or moved_y >= len(pmap[0]) or moved_x < 0 or moved_y < 0:
				p = float("inf")  # outside area
				print("outside region!")
			else:
				p = pmap[moved_x][moved_y]
			if min_potential > p:
				min_potential = p
				min_pot_x = moved_x
				min_pot_y = moved_y
		robot_path.x = min_pot_x
		robot_path.y = min_pot_y
		x_final = robot_path.x * grid_size + minx
		y_final = robot_path.y * grid_size + miny
		d = np.hypot(goal.x - x_final, goal.y - y_final)
		path.append(Node(x_final, y_final))
		lim +=1

	print("Done!!")
	return path, pmap


def main(grid_size, func, ETA, K):
	global obslist
	start = Node(1,1)
	goal = Node(20,20)
	obslist = [(4.5, 3, 2), (3, 12, 2), (15, 15, 3)]  #[(x, y, radius)]
	grid_size = 0.5
	path, pmap = potential_field_planning(start, goal, grid_size, [K, ETA], func)

	"""
	Plotting
	"""
	figure, axes = plt.subplots()
	plt.rcParams["figure.figsize"] = (20,20)

	# plotting the obstacles
	for obs in obslist:
		obstacle = plt.Circle((obs[0], obs[1]), obs[2], color="black", fill=False)
		axes.add_artist(obstacle)

	# plotting path
	plt.plot(1,1,'kp') #start
	plt.plot(20,20,'kp') #goal
	x_cord = []
	y_cord = []
	for v in path:
		x_cord.append(v.x)
		y_cord.append(v.y)
	plt.plot(x_cord, y_cord, "r-", linewidth=1, label='Final Path')

	axes.set_aspect(1)
	plt.xlim(0,30)
	plt.ylim(0,30)
	plt.legend()
	plt.title('Configuration Space')
	plt.savefig("./images/apf.png")
	# plt.show()
	return figure


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Artificial Potential Function')
	parser.add_argument('-g','--grid', type=float,
						help='Grid size; Default=0.5', default=0.5)
	parser.add_argument('-f','--function', type=str,
						help='Attractive Potential function(p=paraboloid, c=conical); Default=c', default='c')
	parser.add_argument('-a','--attractive', type=float,
						help='Attractive Potential Gain; Default=1', default=1)
	parser.add_argument('-r','--repulsive', type=float,
						help='Repuslive Potential Gain; Default=5000', default=5000)
	args = parser.parse_args()

	
	grid_size = args.grid
	func = args.function
	K = args.attractive
	ETA = args.repulsive

	main(grid_size, func, ETA, K)