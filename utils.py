import networkx as nx
nx.__version__
import pickle
import matplotlib.pyplot as plt
from planning_utils import  FLYING_ALTITUDE, SAFETY_DISTANCE, a_star_graph, heuristic
from bresenham import bresenham
import numpy as np
import numpy.linalg as LA


# Getting the largest connected subgraph
def remove_unconnected_subgraphs(Gr):
	Gr = max(nx.connected_component_subgraphs(Gr), key=len)
	return Gr

def get_next_node_in_chain(Gr, node, previous, not_accepted_nodes=[]):
	neighbors = list(Gr.neighbors(node))
	# print('get_next_node_in_chain, node {0}, prev {1}, neighbors {2}, not_accepted {3}'.format(node, previous, neighbors, not_accepted_nodes))
	if len(neighbors) != 2:
		return node

	# We are only interested in nodes with 2 neighbors
	if node not in not_accepted_nodes:
		return node

	# Keep going further
	for neighbor in neighbors:
		if neighbor == previous:
			continue
		return get_next_node_in_chain(Gr, neighbor, node, not_accepted_nodes)

def remove_unnecessary_nodes(Gr, Cg, safety_height):
	nodes_to_remove = []
	edges_to_add = []
	for n in Gr.nodes:
		neighbors = list(Gr.neighbors(n))
		if len(neighbors) == 2:
			left = get_next_node_in_chain(Gr, neighbors[0], n, nodes_to_remove)
			right = get_next_node_in_chain(Gr, neighbors[1], n, nodes_to_remove)

			# Check visible path between left and right
			hit = False
			cells = list(bresenham(int(left[0]), int(left[1]), int(right[0]), int(right[1])))
			for c in cells:
				# First check if we're off the map
				if np.amin(c) < 0 or c[0] >= Cg.shape[0] or c[1] >= Cg.shape[1]:
					hit = True
					break
				# Next check if we're in collision
				if Cg[c[0], c[1]] >= safety_height:
					hit = True
					break

			# If the edge does not hit on obstacle
			# add it to the list
			if not hit:
				dist = LA.norm(np.array(left) - np.array(right))
				edges_to_add.append((left, right, dist))
				nodes_to_remove.append(n)

	for edge in edges_to_add:
		left = edge[0]
		right = edge[1]
		dist = edge[2]
		if left not in nodes_to_remove and right not in nodes_to_remove:
			Gr.add_edge(left, right, weight=dist)

	Gr.remove_nodes_from(nodes_to_remove)
	return Gr

def print_info(Gr, Cg, north_offset, east_offset):
	print('Graph nodes: %5d' % len(Gr.nodes))
	print('Graph edges: %5d' % len(Gr.edges))
	print('Grid dimensions {0}, north_offset: {1}, east_offset: {2} '.format(Cg.shape, north_offset, east_offset))

def load_graph_from_pickle(pkl_filename):
	print('Loading {0} graph'.format(pkl_filename))
	with open(pkl_filename, "rb") as pfile:
		dist_pickle = pickle.load(pfile)

		Gr = dist_pickle['graph']
		Cg = dist_pickle['collision_grid']
		north_offset = dist_pickle['north_offset']
		east_offset = dist_pickle['east_offset']

	return Gr, Cg, north_offset, east_offset

def save_graph_to_pickle(Gr, Cg, north_offset, east_offset, pkl_filename):
	try:
		with open(pkl_filename, 'wb+') as pfile:
			print('Saving to pickle file', pkl_filename)
			pickle.dump(
			{
				'graph': Gr,
				'collision_grid': Cg,
				'north_offset' : north_offset,
				'east_offset' : east_offset,
			},
			pfile, pickle.HIGHEST_PROTOCOL)
	except Exception as e:
		print('Unable to save data to ', pkl_filename, ':', e)

def visualize_graph(Gr, Cg, nmin=0, emin=0):
	# Plot it up!
	fig = plt.figure(figsize=(10,10))
	plt.imshow(Cg, origin='lower', cmap='Greys') 

	# Draw edges in green
	for (n1, n2) in list(Gr.edges)[0:1]:
		plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green', alpha=1)

	# Draw connected nodes in red
	for n1 in list(Gr.nodes)[0:1]:
		print(n1)
		plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

	plt.scatter(0 - emin, 0 - nmin, c='blue') # (0,0)
	plt.scatter(emin - emin, nmin - nmin , c='green') # Lowest point

	plt.xlabel('EAST')
	plt.ylabel('NORTH')
	plt.show()

import sys
def perform_astar(Gr, Cg, nmin=0, emin=0):

	#drone_location = (-emin, -nmin, 5.0) # map coordinates
	drone_location = (445.04762260615826, 315.94609723985195, 5.0)
	print('Find Start node from {0}'.format(drone_location))
	nearest_start = None
	closest_distance = sys.float_info.max
	for n in Gr.nodes:
		# heuristic is the Euclidean distance:
		distance = heuristic(drone_location, n)
		if distance < closest_distance:
			closest_distance = distance
			nearest_start = n

	if nearest_start == None:
		print('Error while getting closest starting node')
		return
	print('Found starting node = {0}'.format(nearest_start))

	##########
	
	goal_location = (240.7685, 360.76114, 5.0) # map coordinates
	print('Find Goal node from {0}'.format(goal_location))
	nearest_goal = None
	closest_distance = sys.float_info.max
	for n in Gr.nodes:
		# heuristic is the Euclidean distance:
		distance = heuristic(goal_location, n)
		if distance < closest_distance:
			closest_distance = distance
			nearest_goal = n

	################
	start = nearest_start
	print('Start: ', start)
	goal = nearest_goal
	print('Goal: ', goal)

	path, cost = a_star_graph(Gr, heuristic, start, goal)
	print(len(path), path)
	if len(path) == 0:
		return

	waypoints = [[p[0], p[1], p[2], 0] for p in path]

	print("start")

	fig = plt.figure(figsize=(10,10))
	plt.imshow(Cg, cmap='Greys', origin='lower')

	path_pairs = zip(waypoints[:-1], waypoints[1:])
	for (n1, n2) in path_pairs:
		plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'green')

	plt.scatter(drone_location[0], drone_location[1], c='blue') # (0,0)
	plt.scatter(emin - emin, nmin - nmin , c='green') # Lowest point
	plt.scatter(100, 0, c='purple') # (0,0)

	plt.xlabel('EAST')
	plt.ylabel('NORTH')
	plt.show()

def create_graph_from_voronoi(voronoi_graph, grid, k=10):
    g = nx.Graph()
    nodes = tuple(map(tuple, voronoi_graph.vertices))
    tree = KDTree(nodes)
    # Check each edge from graph.ridge_vertices for collision
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue

            hit = False
            cells = list(bresenham(int(n1[0]), int(n1[1]), int(n2[0]), int(n2[1])))
            for c in cells:
                # First check if we're off the map
                if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                    hit = True
                    break
                # Next check if we're in collision
                if grid[c[0], c[1]] >= FLYING_ALTITUDE + SAFETY_DISTANCE:
                    hit = True
                    break
            # If the edge does not hit on obstacle
            # add it to the list
            if not hit:
                dist = LA.norm(np.array(n2) - np.array(n1))
                g.add_edge((n1[0], n1[1], FLYING_ALTITUDE), (n2[0], n2[1], FLYING_ALTITUDE), weight=dist)

    return g, tree

from planning_utils import create_grid
from scipy.spatial import Voronoi
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from bresenham import bresenham
if __name__== "__main__":
	test_case = 1
	if test_case == 1:
		print('Voronoi')
		# Unit testing of functions in the file
		Gr, Cg, no, eo = load_graph_from_pickle('graph.voronoi.raw.p')
		print_info(Gr, Cg, no, eo)

		visualize_graph(Gr, Cg)

		Gr = remove_unconnected_subgraphs(Gr)
		print_info(Gr, Cg, no, eo)

		Gr = remove_unnecessary_nodes(Gr, Cg, FLYING_ALTITUDE+SAFETY_DISTANCE)
		print_info(Gr, Cg, no, eo)

		#visualize_graph(Gr, Cg)

		save_graph_to_pickle(Gr, Cg, no, eo, 'graph.voronoi.p')
		perform_astar(Gr, Cg, no, eo)

	elif test_case == 2:
		Gr, Cg, no, eo = load_graph_from_pickle('graph.voronoi.p')
		print_info(Gr, Cg, no, eo)
		
		# Plot it up!
		fig = plt.figure(figsize=(10,10))
		plt.imshow(Cg, origin='lower', cmap='Greys') 

		# Draw edges in green
		#for (n1, n2) in Gr.edges:
		#	plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'green', alpha=1)

		# Draw connected nodes in red
		for n1 in Gr.nodes:
		    plt.scatter(n1[1], n1[0], c='red')

		plt.scatter(0, 0, c='blue')

		plt.xlabel('EAST')
		plt.ylabel('NORTH')
		plt.show()
	elif test_case == 3:
		filename = 'colliders.csv'
		data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

		safety_distance = SAFETY_DISTANCE

		print('Create grid')
		Cg, centers, north_offset, east_offset = create_grid(data, FLYING_ALTITUDE, SAFETY_DISTANCE)
		np_centers = np.array(centers)
		print('Create Voronoi')
		voronoi_graph = Voronoi(np_centers[:,:-1])
		print('Create Graph')
		Gr, tree = create_graph_from_voronoi(voronoi_graph, Cg)

		print_info(Gr, Cg, north_offset, east_offset)

		save_graph_to_pickle(Gr, Cg, north_offset, east_offset, 'graph.voronoi.raw.p')
		visualize_graph(Gr, Cg)

