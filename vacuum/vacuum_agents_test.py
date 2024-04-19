'''
Use this script file to define your robot vacuum agents.

The run function will generate a map showing the animation of the robot, and return the output of the loss function at
the end of the run. The many_runs function will run the simulation multiple times without the visualization and
return the average loss.

You will need to implement a run_all function, which executes the many_runs function for all 12 of your agents (with
the correct parameters) and sums up their returned losses as a single value. Your run_all function should use the
following parameters for each agent: map_width=20, max_steps=50000 runs=100.

'''
import time

from vacuum import *

directions = ['north', 'south', 'east', 'west']

def random_agent(percept, map, agent):
    if (percept):
        return 'clean'

    return random.choice(directions)


import random

# Define colors
yellow = '\033[93m'
green = '\033[92m'
reset = '\033[0m'
blue = '\033[94m'
red = '\033[91m'

def a_star_search(map_, start, goal):
    """
    A* algorithm to find the shortest path from start to goal on the map.

    Args:
      map_: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      start: A tuple representing the starting position (x, y).
      goal: A tuple representing the goal position (x, y).

    Returns:
      A list representing the path from start to goal.
    """
    def heuristic(node):
        if goal is None:
            return 0
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    open_list = [(0, start)]
    heapq.heapify(open_list)
    came_from = {}
    cost_so_far = {start: 0}

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        if current_node == goal:
            break

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            next_node = (current_node[0] + dx, current_node[1] + dy)
            if 0 <= next_node[0] < len(map_) and 0 <= next_node[1] < len(map_[0]) and map_[next_node[0]][next_node[1]] != 'wall':
                new_cost = current_cost + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(next_node)
                    heapq.heappush(open_list, (priority, next_node))
                    came_from[next_node] = current_node

    path = []
    current_node = goal
    while current_node != start:
        if(current_node is None):
            break
        path.append(current_node)
        # print("current_node: ", current_node)
        # print("came_from: ", came_from)
        try:
            current_node = came_from[current_node]
        except:
            return Exception
    path.append(start)
    path.reverse()
    return path


def depth_limited_search(map_, start, depth_limit):
    """
    Perform depth-limited search starting at the given start position.

    Args:
      map_: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      start: A tuple representing the starting position (x, y).
      depth_limit: The maximum depth limit for the search.

    Returns:
      A tuple representing the nearest dirty square (x, y) within the depth limit, or None if not found.
    """
    stack = [(start, 0)]  # Stack to store the current position and depth
    visited = set()

    while stack:
        current_pos, depth = stack.pop()
        x, y = current_pos
        visited.add((x, y))

        if map_[x][y] == 'dirt':
            return x, y  # Found a dirty square within the depth limit

        if depth < depth_limit:
            # Add adjacent clean squares to the stack
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                new_x, new_y = x + dx, y + dy
                if 0 <= new_x < len(map_) and 0 <= new_y < len(map_[0]) and (new_x, new_y) not in visited and map_[new_x][new_y] != 'wall':
                    stack.append(((new_x, new_y), depth + 1))

    # If no dirty square is found within the depth limit
    return None


def bfs(map_, start):
    """
    Perform breadth-first search starting at the given start position.

    Args:
      map_: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      start: A tuple representing the starting position (x, y).

    Returns:
      A tuple representing the nearest dirty square (x, y).
    """
    queue = [start]
    visited = set()

    while queue:
        x, y = queue.pop(0)
        visited.add((x, y))

        # Check if the current square is dirty
        if map_[x][y] == 'dirt':
            return x, y

        # Add adjacent clean squares to the queue
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = x + dx, y + dy
            if (0 <= new_x < len(map_) and 0 <= new_y < len(map_[0]) and
                    (new_x, new_y) not in visited and map_[new_x][new_y] != 'wall'):
                queue.append((new_x, new_y))
    # If no dirty square is found
    return None


def print_map(map_, agent = (-1, -1)):
    print(agent)
    transposed = [[row[i] for row in map_] for i in range(len(map_[0]))]
    transposed = transposed[::-1]
    if(agent[0] != -1):
        transposed[agent[0]][agent[1]] = 'start'
    # Display the map with colors
    for row in transposed:
        for cell in row:
            if cell == 'dirt':
                print(yellow + 'd', end=" ")
            elif cell == 'clean':
                print(yellow + ' ', end=" ")
            elif cell == 'focus':
                print(blue + 'f', end=" ")
            elif cell == 'start':
                print(red + 's', end=" ")
            else:
                print(green + 'X', end=" ")
        print("")



"""
    build map as moving, a* to questionmake/dirty
"""

def entire_map_no_memory_actions(percept, map, agent):
    """
    This agent has knowledge of the entire map but no memory of its past location.
    It chooses the next move to minimize the number of dirty squares remaining
    (Actions loss function).

    Args:
      percept: a boolean indicating dirt (True).
      map: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      agent: A tuple representing the current x,y of the agent.

    Returns:
      A string representing the next action (direction to move) for the robot.
    """
    if percept:
        return 'clean'

    # Get the current location.
    x, y = agent

    # if adjacent square is dirty, clean it
    if y < len(map[0]) - 1:  # North
        if map[x][y + 1] == 'dirt':
            return 'north'
    if x < len(map) - 1:
        if map[x + 1][y] == 'dirt':
            return 'east'
    if y > 0:
        if map[x][y - 1] == 'dirt':
            return 'south'
    if x > 0:
        if map[x - 1][y] == 'dirt':
            return 'west'

    # Find the closest dirty square using A*
    dirty_squares = [(i, j) for i in range(len(map)) for j in range(len(map[0])) if map[i][j] == 'dirt']
    #closest Node
    closest = min(dirty_squares, key=lambda c: abs(c[0] - x) + abs(c[1] - y))
    # print ("closest: ", closest)
    closest_dirty_squareDLS = closest



    if dirty_squares:
        # closest_dirty_square = bfs(map, (x, y))
        # closest_dirty_squareDLS = depth_limited_search(map, (x,y), len(map) * len(map[0]))
        # closest_dirty_squareDLS = depth_limited_search(map, (0,0), len(map) * len(map[0]))
        # print("closest dirty square: ", closest_dirty_squareDLS)
        # print("closest dirty square: ", closest_dirty_square)
        # time.sleep(1)
        try:
            path_to_dirty_square = a_star_search(map, (x, y), closest_dirty_squareDLS)
        except:
            #there's an unreachable dirty square
            closest_dirty_squareDLS = depth_limited_search(map, (0, 0), len(map) * len(map[0]))
            try:
                path_to_dirty_square = a_star_search(map, (x, y), closest_dirty_squareDLS)
            except:
                closest_dirty_squareDLS = depth_limited_search(map, (int(len(map)/2), int(len(map[0])/2)), len(map) * len(map[0]))
                path_to_dirty_square = a_star_search(map, (x, y), closest_dirty_squareDLS)

        # Determine the next action based on the path
        try:
            next_x, next_y = path_to_dirty_square[1]  # Next position after current position
        except:
            return 'clean'
        if next_x == x + 1:
            return 'east'
        elif next_x == x - 1:
            return 'west'
        elif next_y == y + 1:
            return 'north'
        elif next_y == y - 1:
            return 'south'

    # If there are no dirty squares left, stop cleaning
    return 'clean'

def entire_map_no_memory_dirt(percept, map, agent):
    """
    This agent has knowledge of the entire map but no memory of its past location.
    It chooses the next move to minimize the number of dirty squares remaining
    (Actions loss function).

    Args:
      percept: a boolean indicating dirt (True).
      map: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      agent: A tuple representing the current x,y of the agent.

    Returns:
      A string representing the next action (direction to move) for the robot.
    """
    if percept:
        return 'clean'

    # Get the current location.
    x, y = agent
    #
    # print()
    # print()
    # print_map(map)

    #if adjacent square is dirty, clean it
    if y < len(map[0]) - 1:  # North
        if map[x][y + 1] == 'dirt':
            return 'north'
    if x < len(map) - 1:
        if map[x + 1][y] == 'dirt':
            return 'east'
    if y > 0:
        if map[x][y - 1] == 'dirt':
            return 'south'
    if x > 0:
        if map[x - 1][y] == 'dirt':
            return 'west'

    # Find the closest dirty square using A*
    dirty_squares = [(i, j) for i in range(len(map)) for j in range(len(map[0])) if map[i][j] == 'dirt']
    if dirty_squares:
        closest_dirty_squareDLS = depth_limited_search(map, (0, 0), len(map) * len(map[0]))

        path_to_dirty_square = a_star_search(map, (x, y), closest_dirty_squareDLS)
        # print_map(map, (x,y))
        # print("path_to_dirty_square: ", path_to_dirty_square)

        # Determine the next action based on the path
        if(len(path_to_dirty_square) <= 1):
            return 'clean'
        next_x, next_y = path_to_dirty_square[1]  # Next position after current position
        if next_x == x + 1:
            return 'east'
        elif next_x == x - 1:
            return 'west'
        elif next_y == y + 1:
            return 'north'
        elif next_y == y - 1:
            return 'south'

    # If there are no dirty squares left, stop cleaning
    return 'clean'


# MAKE SURE TO CHANGE FOR DIFFERENT MAPSIZE
agentBuiltMap = [['unknown' for i in range(40)] for j in range(40)]
agentBuiltMap[39][39] = 'self'
agentBuiltXY = (39, 39)




surroundingStack = []
def neighbors_memory_actions(percept, map, agent):
    if percept:
        return 'clean'
    x, y = agent
    width = len(map)
    height = width
    if y + 1 < height and map[x][y + 1] == 'dirt':
        surroundingStack.append('south')
        return 'north'
    elif x + 1 < width and map[x + 1][y] == 'dirt':
        surroundingStack.append('west')
        return 'east'
    elif y - 1 >= 0 and map[x][y - 1] == 'dirt':
        surroundingStack.append('north')
        return 'south'
    elif x - 1 >= 0 and map[x - 1][y] == 'dirt':
        surroundingStack.append('east')
        return 'west'
    else:
        if not surroundingStack:
            return random.choice(directions)
        return surroundingStack.pop()

def neighbors_no_memory_actions(percept, map, agent):
    """
    This agent has knowledge of the adjacent squares but no memory of its past location.
    It chooses the next move to minimize the number of dirty squares remaining
    (Actions loss function).

    Args:
      percept: a boolean indicating dirt (True).
      map: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      agent: A tuple representing the current x,y of the agent.

    Returns:
      A string representing the next action (direction to move) for the robot.
    """
    if percept:
        return 'clean'

    # Get the current location.
    x, y = agent

    #if adjacent square is dirty, clean it
    if y < len(map[0]) - 1:  # North
        if map[x][y + 1] == 'dirt':
            return 'north'
    if x > 0:
        if map[x - 1][y] == 'dirt':
            return 'west'
    if y > 0:
        if map[x][y - 1] == 'dirt':
            return 'south'
    if x < len(map) - 1:
        if map[x + 1][y] == 'dirt':
            return 'east'

    #if no adjacent square is dirty, move to a random square
    return random.choice(directions)


import heapq

def heuristic_distance(pos1, pos2):
    """
    Calculate the Manhattan distance between two positions.

    Args:
      pos1: A tuple representing the position (x, y).
      pos2: A tuple representing the position (x, y).

    Returns:
      The Manhattan distance between the two positions.
    """
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def is_valid_move(map_, pos):
    """
    Check if the move is valid (not a wall).

    Args:
      map_: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      pos: A tuple representing the position (x, y).

    Returns:
      True if the move is valid, False otherwise.
    """
    x, y = pos
    return 0 <= x < len(map_) and 0 <= y < len(map_[0]) and map_[x][y] != 'wall'

def find_shortest_path(map_, agent):
    """
    Find the shortest path to visit all dirty squares while avoiding walls.

    Args:
      map_: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      agent: A tuple representing the current position (x, y) of the agent.

    Returns:
      A list of tuples representing the shortest path to visit all dirty squares.
    """
    dirty_squares = [(i, j) for i in range(len(map_)) for j in range(len(map_[0])) if map_[i][j] == 'dirt']
    if not dirty_squares:
        return []  # No dirty squares to clean

    print("dirtySquares", len(dirty_squares), dirty_squares)
    start = agent
    visited = set()
    pq = [(0, [start])]  # Priority queue to store the path with its cost

    while pq:
        cost, path = heapq.heappop(pq)
        current_pos = path[-1]
        print(cost, len(path), path, current_pos)

        time.sleep(1)

        if current_pos in visited:
            continue
        visited.add(current_pos)

        if len(path) == len(dirty_squares) + 1:
            print("Shortest path: ", path)
            return path  # All dirty squares visited

        for neighbor in [(current_pos[0] + dx, current_pos[1] + dy) for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]]:
            if is_valid_move(map_, neighbor) and neighbor not in visited:
                new_cost = cost + heuristic_distance(current_pos, neighbor)
                new_path = path + [neighbor]
                heapq.heappush(pq, (new_cost, new_path))


    return []  # No valid path found


def entire_map_memory_actions(percept, map, agent):
    """
    This agent has knowledge of the entire map and memory of its past location.
    It chooses the next move to minimize the number of dirty squares remaining
    (Actions loss function).

    Args:
      percept: a boolean indicating dirt (True).
      map: A 2D list representing the entire map with squares marked as 'dirty', 'wall', or 'clean'.
      agent: A tuple representing the current x,y of the agent.

    Returns:
      A string representing the next action (direction to move) for the robot.
    """
    if percept:
        return 'clean'

    # Get the current location.
    x, y = agent

    # Find the shortest path to visit all dirty squares
    if not entire_map_memory_actions.path:
        entire_map_memory_actions.path = find_shortest_path(map, agent)

    print(entire_map_memory_actions.path)
    if not entire_map_memory_actions.path:
        return 'clean'

    # Determine the next action based on the path
    next_x, next_y = entire_map_memory_actions.path[1]  # Next position after current position
    if next_x == x + 1:
        return 'east'
    elif next_x == x - 1:
        return 'west'
    elif next_y == y + 1:
        return 'north'
    elif next_y == y - 1:
        return 'south'

    # If there are no dirty squares left, stop cleaning
    return 'clean'

entire_map_memory_actions.path = []

"""
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                                        Agents
########################################################################################################################
                                                         Runs
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
"""


# input args for run: map_width, max_steps, agent_function, loss_function

# run(20, 50000, random_agent, 'actions')
# run(20, 50000, entire_map_no_memory_actions, 'actions')
# run(20, 50000, entire_map_no_memory_actions, 'actions')
# run(20, 50000, entire_map_memory_dirt, 'dirt')
# run(20, 50000, entire_map_no_memory_dirt, 'dirt')

## input args for many_runs: map_width, max_steps, runs, agent_function, loss_function
#
# print(many_runs(20, 50000, 100, entire_map_no_memory_actions, 'actions'))
# print(many_runs(20, 5000, 10, neighbors_no_memory_actions, 'dirt'),
#       many_runs(20, 5000, 10, neighbors_no_memory_actions, 'actions'))
# print(many_runs(20, 5000, 10, entire_map_no_memory_dirt, 'dirt'),
#       many_runs(20, 5000, 10, entire_map_no_memory_dirt, 'actions'))
# print(many_runs(20, 5000, 10, entire_map_no_memory_actions, 'dirt'),
#       many_runs(20, 5000, 10, entire_map_no_memory_actions, 'actions'))
# print(many_runs(20, 5000, 100, entire_map_no_memory_actions, 'dirt'))
# print(many_runs(20, 50000, 100, entire_map_no_memory_dirt, 'dirt'))

def run_all():
    """
    Run all 12 agents and sum up their losses.

    Returns:
      The total loss of all agents combined.
    """
    # Define the agents

    """
    Best Agents
        entire map No memory actions :  entire_map_no_memory_actions
        entire map No memory dirt :     entire_map_no_memory_dirt
        entire map Memory actions :     entire_map_no_memory_actions
        entire map Memory dirt :        entire_map_no_memory_dirt
        
        Neighbors No memory actions :   neighbors_no_memory_actions
    """
    agents = [
        entire_map_no_memory_actions,
        entire_map_no_memory_dirt,
        neighbors_no_memory_actions,
        neighbors_no_memory_actions,
        neighbors_no_memory_actions,
        neighbors_no_memory_actions,
        neighbors_no_memory_actions,
        neighbors_no_memory_actions,
        neighbors_memory_actions,
        # random_agent
    ]

    # Run each agent and sum up their losses
    print(f'{"Agent": <60}', 'Dirt Loss\tActions Loss')
    for agent in agents:
        loss_dirt = many_runs(20, 5000, 100, agent, 'dirt')
        loss_actions = many_runs(20, 5000, 100, agent, 'actions')

        print(f'{agent.__name__: <60}{loss_dirt}, \t{loss_actions}')


run_all()
