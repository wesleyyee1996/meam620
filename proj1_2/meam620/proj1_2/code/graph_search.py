from heapq import heappush, heappop  # Recommended.
import numpy as np
from node import Node
import heapq

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)

    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    # Initialize the start and goal node
    start_node = Node(g=0, h=0, index=start_index, parent=None)
    goal_node = Node(g=0, h=0, index=goal_index, parent=None)

    # Initialize the closed and open dicts which will allow for O(1) index-node lookup
    closed = {}
    open_dict = {start_index: start_node}

    # Initialize heap of nodes to explore
    open = [start_node]

    curr_node = None
    while not goal_node.is_closed:              # keep iterating until the goal node is closed
        try:
            curr_node = heapq.heappop(open)     # keep popping node with smallest cost to start node
        except IndexError:                      # if queue is empty then exit. most likely means no solution
            break

        if curr_node.is_closed: continue        # since we don't want to delete from heap cause that means we will need
                                                # to resort heap, just move on if a node is already closed
        curr_node.is_closed = True              # set current node to be closed
        closed[curr_node.index] = curr_node     # also indicate this in the closed hashmap

        for idx, cost in get_neighbors(curr_node, occ_map.map).items():
            if idx not in closed:
                neighbor = Node(g=cost, h=calc_heuristic(astar, idx, goal_index), index=idx, parent=curr_node)
                if idx == goal_node.index:
                    goal_node = neighbor
                if idx not in open_dict or neighbor.g < open_dict[idx].g:
                    heappush(open, neighbor)
                    open_dict[idx] = neighbor

    path = []
    curr_node = goal_node
    while curr_node != start_node:                                      # create path now, starting from goal and
        if (curr_node.parent is None):                                  # retrace steps until start
            break
        path.append(occ_map.index_to_metric_center(curr_node.index))
        curr_node = curr_node.parent

    path = np.array(path)
    if len(path)==1:
        path = None

    

    # Return a tuple (path, nodes_expanded)
    return (path, len(closed))

def calc_heuristic(a_star, curr_idx, goal_idx):
    if a_star:
        return np.linalg.norm([goal_idx[0]-curr_idx[0], goal_idx[1]-curr_idx[1], goal_idx[2]-curr_idx[2]])
    else:
        return 0

def get_neighbors(node, map):
    """
    Returns all neighbors possible for given node
    """
    list_neighbors = {}
    for i in range(node.index[0]-1, node.index[0]+2):
        for j in range(node.index[1]-1, node.index[1]+2):
            for k in range(node.index[2]-1, node.index[2]+2):
                if (0 <= i < map.shape[0]) \
                        and (0 <= j < map.shape[1]) \
                        and (0 <= k < map.shape[2]):
                    if (i,j,k) != node.index:
                        if map[i][j][k] == False:
                            cost = np.linalg.norm([i - node.index[0],
                                                   j - node.index[1],
                                                   k - node.index[2]])
                            list_neighbors[(i,j,k)] = cost+node.g
    return list_neighbors

