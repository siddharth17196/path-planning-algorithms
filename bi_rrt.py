#!/usr/bin/env python
# coding: utf-8

import matplotlib.pyplot as plt
import math
import numpy as np
import random
import argparse


class Node:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.path_so_far = []


def get_random_point(lc=0, rc=30):
	rand_point = Node(random.uniform(lc,rc),random.uniform(lc, rc))
	return rand_point

def merge_intersetion(node1, node2):
	global obslist
	for obs in obslist:
		dr = np.hypot(node2.x-node1.x, node2.y-node1.y)
		D = (node1.x-obs[0])*(node2.y-obs[1]) - (node2.x-obs[0])*(node1.y-obs[1])
		d = (obs[2]*dr)**2 - D**2
		if d >= 0:
			return False
	return True

def get_nearest_point(point, tree):
	distance_list = [np.hypot(node.x - point.x, node.y - point.y)**2 for node in tree]
	return distance_list.index(min(distance_list))

def get_new_point(rnd_point, old_point, delta=1):
	theta = math.atan((old_point.y-rnd_point.y)/(old_point.x-rnd_point.x))
	temp_p1 = Node(delta*math.cos(theta) + old_point.x, delta*math.sin(theta) + old_point.y)
	temp_p2 = Node(-delta*math.cos(theta) + old_point.x, -delta*math.sin(theta) + old_point.y)
	temp_tree = [temp_p1, temp_p2]
	new_node = temp_tree[get_nearest_point(rnd_point, temp_tree)]
	val = merge_intersetion(new_node, old_point)
	if val and new_node.x >= 0 and new_node.y >=0 and new_node.x <= 30 and new_node.y <=30:
		return new_node
	return False

def add_point(tree):
	rand_point = get_random_point()
	min_dist_index = get_nearest_point(rand_point, tree)
	old_point = tree[min_dist_index]
	new_node = get_new_point(rand_point, old_point)
	if new_node:
		tree.append(new_node)
		new_node.path_so_far = old_point.path_so_far + [new_node]

def merge(tree1, tree2):
	flag = 0
	nl = []
	for node in tree1[::-1]:
		for n in tree2[::-1]:
			if merge_intersetion(node, n):
				nl.append((node, n, len(node.path_so_far) + len(n.path_so_far)))
	if nl != []:
		nl.sort(key=lambda x: x[2])
		path = nl[0][0].path_so_far + nl[0][1].path_so_far[::-1]
		print("Done!!")
	else:
		print("No path found")
		path = None
	return path

def main(max_iter):
	global obslist
	obslist = [(4.5, 3, 2), (3, 12, 2), (15, 15, 3)]  #[(x, y, radius)]
	start_node = Node(1,1)
	start_node.path_so_far.append(start_node)
	goal_node = Node(20,20)
	goal_node.path_so_far.append(goal_node)
	start_tree = [start_node]
	goal_tree = [goal_node]
	for i in range(max_iter):
		add_point(start_tree)
		add_point(goal_tree)
	path = merge(start_tree, goal_tree)

	"""
	Plotting
	"""
	if path:
		figure, axes = plt.subplots()
		plt.rcParams["figure.figsize"] = (15,15)

		# plotting the obstacles
		for obs in obslist:
			obstacle = plt.Circle((obs[0], obs[1]), obs[2], color="black")
			axes.add_artist(obstacle)

		# plotting both the trees
		trees = [start_tree, goal_tree]
		color = ["blue", "green"]
		lab = ["Tree from start point", "Tree from end point"]
		for i,tree in enumerate(trees):
			x_cord = []
			y_cord = []
			for v in tree:
					x_cord.append(v.x)
					y_cord.append(v.y)
			plt.scatter(x_cord, y_cord, c=[color[i]], label=lab[i])

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
		plt.savefig("./images/bi_rrt.png")
		# plt.show()
		return figure


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Bi-Directional RRT')
	parser.add_argument('-i','--iter', type=int,
						help='Number of iterations (integer); Default=100', default=100)
	args = parser.parse_args()
	max_iter = args.iter

	main(max_iter)
