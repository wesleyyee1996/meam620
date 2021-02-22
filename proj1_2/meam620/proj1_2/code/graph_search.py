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

    start_node = Node(g=0, h=0, index=start_index, parent=None)
    goal_node = Node(g=0, h=0, index=goal_index, parent=None)

    closed = {}
    open_dict = {start_index: start_node}
    open = [start_node]

    curr_node = None
    while not goal_node.is_closed:
        try:
            curr_node = heapq.heappop(open)
        except IndexError:
            break

        if curr_node.is_closed: continue
        curr_node.is_closed = True
        closed[curr_node.index] = curr_node

        for idx, cost in get_neighbors(curr_node, occ_map.map).items():
            if idx not in closed:
                neighbor = Node(g=cost, h=0, index=idx, parent=curr_node)
                if idx == goal_node.index:
                    goal_node = neighbor
                if idx not in open_dict:
                    heappush(open, neighbor)
                    open_dict[idx] = neighbor
                elif neighbor < open_dict.get(idx):
                    open_dict[idx] = neighbor

    path = []
    curr_node = goal_node
    while curr_node != start_node:
        if (curr_node.parent is None):
            break
        path.append(occ_map.index_to_metric_center(curr_node.index))
        curr_node = curr_node.parent

    path = np.array(path)

    # Return a tuple (path, nodes_expanded)
    return (path, len(closed))


def get_neighbors(node, map):
    """
    Returns all neighbors possible for given node
    """
    list_neighbors = {}
    for i in range(node.index[0]-1, node.index[0]+2):
        for j in range(node.index[1]-1, node.index[1]+2):
            for k in range(node.index[2]-1, node.index[2]+2):
                if (0 <= i < map.shape[0]) and (0 <= j < map.shape[1]) and (0 <= k < map.shape[2]):
                    if (i,j,k) != node.index:
                        if map[i][j][k] == False:
                            cost = np.linalg.norm([i - node.index[0], j - node.index[1], k - node.index[2]])
                            list_neighbors[(i,j,k)] = cost+node.f
    return list_neighbors

