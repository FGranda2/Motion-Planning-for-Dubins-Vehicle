"""
Assignment #2 Template file
"""
import random
import math
import numpy as np
"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees* (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_star_planner.py. Your
   implementation can be tested by running RRT_DUBINS_PROBLEM.PY (check the 
   main function).
2. Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
   There are plenty of helper function in the class to ease implementation.
3. Your solution must meet all the conditions specificed below.
4. Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random iterations
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.
2. The planning function must return a list of nodes that represent a collision-free path
   from start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the node class to understand the terminology)
3. The returned path should have the start node at index 0 and goal node at index -1,
   while the parent node for node i from the list should be node i-1 from the list, ie,
   the path should be a valid list of nodes.
   (READ the documentation of the node to understand the terminology)
4. The node locations must not lie outside the map boundaries specified by
   RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1. Do not rename the file rrt_star_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def rrt_star_planner(rrt_dubins, display_map=False):
    """
        Execute RRT* planning using Dubins-style paths. Make sure to populate the node_list.

        Inputs
        -------------
        rrt_dubins  - (RRT_DUBINS_PROBLEM) Class conatining the planning
                      problem specification
        display_map - (boolean) flag for animation on or off (OPTIONAL)

        Outputs
        --------------
        (list of nodes) This must be a valid list of connected nodes that form
                        a path from start to goal node

        NOTE: In order for rrt_dubins.draw_graph function to work properly, it is important
        to populate rrt_dubins.nodes_list with all valid RRT nodes.
    """
    # LOOP for max iterations
    i = 0
    # Threshold distance value for sampling close to goal
    bound_th = 6
    # Set seed value for random function
    random.seed(a=155)
    # Gamma value for ball radius
    gamma = 20
    while i < rrt_dubins.max_iter:
        i += 1
        # Introduce 10% bias of sampling close to goal
        # prob=0 sample close to goal | prob=1 sample elsewhere
        prob = np.random.choice(2, 1, p=[0.1, 0.9])
        if prob == 0:
            # Set limits not to sample outside map
            xneg_lim = max([rrt_dubins.x_lim[0], rrt_dubins.goal.x - bound_th / 2])
            xpos_lim = min([rrt_dubins.x_lim[1], rrt_dubins.goal.x + bound_th / 2])
            yneg_lim = max([rrt_dubins.y_lim[0], rrt_dubins.goal.y - bound_th / 2])
            ypos_lim = min([rrt_dubins.y_lim[1], rrt_dubins.goal.y + bound_th / 2])
            # Generate a random vehicle state (x, y, yaw)
            x_state = random.uniform(xneg_lim, xpos_lim)
            y_state = random.uniform(yneg_lim, ypos_lim)
            yaw_state = random.uniform(rrt_dubins.goal.yaw - np.pi / (bound_th / 2),
                                       rrt_dubins.goal.yaw + np.pi / (bound_th / 2))
        else:
            # Generate a random vehicle state (x, y, yaw)
            x_state = random.uniform(rrt_dubins.x_lim[0], rrt_dubins.x_lim[1])
            y_state = random.uniform(rrt_dubins.y_lim[0], rrt_dubins.y_lim[1])
            yaw_state = random.uniform(0, 2 * np.pi)

        # Create node for random vehicle state
        node_state = rrt_dubins.Node(x_state, y_state, yaw_state)
        # Find an existing node nearest to the random vehicle state
        cost_list = []
        for idx in range(len(rrt_dubins.node_list)):
            cost = calc_dist_to_node(node_state, rrt_dubins.node_list[idx])
            cost_list.append(cost)

        # Select lower cost/distance = nearest node
        index_min = min(range(len(cost_list)), key=cost_list.__getitem__)

        # Add new node
        new_node = rrt_dubins.propogate(rrt_dubins.node_list[index_min], node_state)

        # Check if the path between nearest node and random state has obstacle collision
        if rrt_dubins.check_collision(new_node):
            # Look for near nodes within radius r
            near_list = nearest_nodes(new_node, rrt_dubins.node_list, gamma)
            # Look for best parent in near list
            test_best_node = best_parent(new_node, rrt_dubins, near_list)
            if test_best_node:
                # Create new_node with best parent
                new_node = test_best_node
            # Add node to list
            rrt_dubins.node_list.append(new_node)
            # Rewire function to improve local connections
            rewire(rrt_dubins, near_list, new_node)

        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()

        # Check if new_node is close to goal for closing
        cost2goal = rrt_dubins.calc_dist_to_goal(rrt_dubins.node_list[-1].x, rrt_dubins.node_list[-1].y)
        # Set threshold distance value for being close to goal
        th_val = 1.5
        close = False
        end_node = rrt_dubins.propogate(rrt_dubins.node_list[-1], rrt_dubins.goal)
        # Check for collisions from last node to goal
        if cost2goal <= th_val and rrt_dubins.check_collision(end_node):
            close = True

        if close:
            print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
            break

    if i == rrt_dubins.max_iter:
        close = False
        path_list = []
        print('reached max iterations')

        # Return path, which is a list of nodes leading to the goal...
    if close:
        # Connect to goal directly
        end_node = rrt_dubins.propogate(rrt_dubins.node_list[-1], rrt_dubins.goal)
        rrt_dubins.node_list.append(end_node)
        # Trace path by using node parents until reaching start node
        back2start = False
        current_node = rrt_dubins.node_list[-1]
        path_list = [current_node]
        while not back2start:
            element = find_element_in_list(current_node.parent, rrt_dubins.node_list)
            if element == 0:
                current_node = rrt_dubins.node_list[element]
                path_list.append(current_node)
                break
            else:
                current_node = rrt_dubins.node_list[element]
                path_list.append(current_node)

        # Reverse list to respect order
        path_list.reverse()

    # Return the node list
    return path_list


# Function to find parent of node in node list
def find_element_in_list(element, list_element):
    try:
        index_element = list_element.index(element)
        return index_element
    except ValueError:
        return None


# Function to compute distance from one node to another
def calc_dist_to_node(from_node, to_node):
    dx = from_node.x - to_node.x
    dy = from_node.y - to_node.y
    return math.hypot(dx, dy)


# Function to find nearest nodes within ball of radius r
def nearest_nodes(from_node, node_list, gamma):
    nearest_list = []
    # Define radius r
    n_nodes = len(node_list) + 1
    r = gamma * np.sqrt((np.log(n_nodes) / n_nodes))
    for idx in range(len(node_list)):
        cost = calc_dist_to_node(from_node, node_list[idx])
        if cost <= r:
            nearest_list.append(idx)
    return nearest_list


# Function to find best parent node to connect
def best_parent(from_node, rrt_dubins, near_list):
    best_cost = np.inf
    best_node = []
    if near_list:
        for idx in range(len(near_list)):
            # Check for collisions
            new_node = rrt_dubins.propogate(rrt_dubins.node_list[near_list[idx]], from_node)
            if rrt_dubins.check_collision(new_node):
                # Check if cost is lower than previous best cost
                new_cost = rrt_dubins.calc_new_cost(rrt_dubins.node_list[near_list[idx]], new_node)
                if new_cost < best_cost:
                    best_cost = new_cost
                    best_node = near_list[idx]

        # Create node with best parent
        if best_node:
            newer_node = rrt_dubins.propogate(rrt_dubins.node_list[best_node], from_node)
        else:
            newer_node = []
    else:
        newer_node = []

    return newer_node


def rewire(rrt_dubins, near_list, new_node):
    if near_list:
        # Look in list of near nodes
        for idx in range(len(near_list)):
            # Check if cost is lower by using new_node as parent
            original_cost = rrt_dubins.node_list[near_list[idx]].cost
            temp_node = rrt_dubins.propogate(new_node, rrt_dubins.node_list[near_list[idx]])
            # Use only if not collisions
            if rrt_dubins.check_collision(temp_node):
                if temp_node.cost < original_cost:
                    # Change parent node to the new_node
                    rrt_dubins.node_list[near_list[idx]].parent = new_node

    return None

