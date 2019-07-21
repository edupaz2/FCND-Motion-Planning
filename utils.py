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

def print_info(Gr, Cg):
	print('Graph nodes: %5d' % len(Gr.nodes))
	print('Graph edges: %5d' % len(Gr.edges))
	print('Grid dimensions: ', Cg.shape)

def load_graph_from_pickle(pkl_filename):
	print('Loading {0} graph'.format(pkl_filename))
	with open(pkl_filename, "rb") as pfile:
		dist_pickle = pickle.load(pfile)

		Gr = dist_pickle['graph']
		Cg = dist_pickle['collision_grid']

	return Gr, Cg

def save_graph_to_pickle(Gr, Cg, pkl_filename):
	try:
		with open(pkl_filename, 'wb+') as pfile:
			print('Saving to pickle file', pkl_filename)
			pickle.dump(
			{
				'graph': Gr,
				'collision_grid': Cg,
			},
			pfile, pickle.HIGHEST_PROTOCOL)
	except Exception as e:
		print('Unable to save data to ', pkl_filename, ':', e)

def visualize_graph(Gr, Cg, figsize_=(10,10)):
	# Plot it up!
	fig = plt.figure(figsize=figsize_)
	plt.imshow(Cg, origin='lower', cmap='Greys') 

	# Draw edges in green
	for (n1, n2) in Gr.edges:
		plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'green', alpha=1)

	# Draw connected nodes in red
	for n1 in Gr.nodes:
	    plt.scatter(n1[1], n1[0], c='red')

	plt.xlabel('EAST')
	plt.ylabel('NORTH')
	plt.show()


def perform_astar(Gr, Cg, figsize_=(10,10), nmin=0, emin=0):
	rnd = np.random.randint(len(Gr.nodes))
	start = list(Gr.nodes)[rnd]
	print('Start: ', rnd)
	rnd = np.random.randint(len(Gr.nodes))
	print('Goal: ', rnd)
	goal = list(Gr.nodes)[rnd]

	path, cost = a_star_graph(Gr, heuristic, start, goal)
	print(len(path), path)

	print("start")

	fig = plt.figure(figsize=figsize_)
	plt.imshow(Cg, cmap='Greys', origin='lower')

	path_pairs = zip(path[:-1], path[1:])
	for (n1, n2) in path_pairs:
		plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')

	plt.xlabel('NORTH')
	plt.ylabel('EAST')
	plt.show()


if __name__== "__main__":
	print('Voronoi')
	# Unit testing of functions in the file
	Gr, Cg = load_graph_from_pickle('graph.voronoi.raw.p')
	print_info(Gr, Cg)

	#visualize_graph(Gr, Cg)

	Gr = remove_unconnected_subgraphs(Gr)
	print_info(Gr, Cg)

	Gr = remove_unnecessary_nodes(Gr, Cg, FLYING_ALTITUDE+SAFETY_DISTANCE)
	print_info(Gr, Cg)

	#visualize_graph(Gr, Cg)

	save_graph_to_pickle(Gr, Cg, 'graph.voronoi.p')
	perform_astar(Gr, Cg)

