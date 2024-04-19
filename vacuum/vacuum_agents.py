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

# Define colors
yellow = '\033[93m'
green = '\033[92m'
reset = '\033[0m'
blue = '\033[94m'
red = '\033[91m'

"""
########################################################################################################################
																													Utils
########################################################################################################################
"""


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
			if 0 <= next_node[0] < len(map_) and 0 <= next_node[1] < len(map_[0]) and map_[next_node[0]][
				next_node[1]] != 'wall':
				new_cost = current_cost + 1
				if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
					cost_so_far[next_node] = new_cost
					priority = new_cost + heuristic(next_node)
					heapq.heappush(open_list, (priority, next_node))
					came_from[next_node] = current_node

	path = []
	current_node = goal
	while current_node != start:
		if current_node is None:
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
				if 0 <= new_x < len(map_) and 0 <= new_y < len(map_[0]) and (new_x, new_y) not in visited and map_[new_x][
					new_y] != 'wall':
					stack.append(((new_x, new_y), depth + 1))

	# If no dirty square is found within the depth limit
	return None


def print_map(map_):
	transposed = [[row[i] for row in map_] for i in range(len(map_[0]))]
	transposed = transposed[::-1]

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


def calculate_next_position(direction, position):
	x, y = position
	if direction == 'north':
		y += 1
	elif direction == 'east':
		x += 1
	elif direction == 'south':
		y -= 1
	elif direction == 'west':
		x -= 1
	return x, y


'''
########################################################################################################################
																										Entire Map
########################################################################################################################
'''


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

	dirty_squares = [(i, j) for i in range(len(map)) for j in range(len(map[0])) if map[i][j] == 'dirt']
	# closest Node
	closest = min(dirty_squares, key=lambda c: abs(c[0] - x) + abs(c[1] - y))

	if dirty_squares:
		try:
			path_to_dirty_square = a_star_search(map, (x, y), closest)
		except:
			# there's an unreachable dirty square
			closest = depth_limited_search(map, (0, 0), len(map) * len(map[0]))
			try:
				path_to_dirty_square = a_star_search(map, (x, y), closest)
			except:
				closest = depth_limited_search(map, (int(len(map) / 2), int(len(map[0]) / 2)), len(map) * len(map[0]))
				path_to_dirty_square = a_star_search(map, (x, y), closest)

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


'''
########################################################################################################################
																													Neighbors
########################################################################################################################
'''

frontier = []


def neighbors_memory_actions(percept, map, agent):
	if percept:
		return 'clean'
	x, y = agent
	width = len(map)
	height = width
	if y + 1 < height and map[x][y + 1] == 'dirt':
		frontier.append('south')
		return 'north'
	elif x + 1 < width and map[x + 1][y] == 'dirt':
		frontier.append('west')
		return 'east'
	elif y - 1 >= 0 and map[x][y - 1] == 'dirt':
		frontier.append('north')
		return 'south'
	elif x - 1 >= 0 and map[x - 1][y] == 'dirt':
		frontier.append('east')
		return 'west'
	else:
		if not frontier:
			return random.choice(directions)
		return frontier.pop()


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

	# if adjacent square is dirty, clean it
	local_directions = []
	if y < len(map[0]) - 1:  # North
		if map[x][y + 1] == 'dirt':
			local_directions.append('north')
	if x < len(map) - 1:
		if map[x + 1][y] == 'dirt':
			local_directions.append('east')
	if y > 0:
		if map[x][y - 1] == 'dirt':
			local_directions.append('south')
	if x > 0:
		if map[x - 1][y] == 'dirt':
			local_directions.append('west')
	if len(local_directions) > 0:
		return random.choice(local_directions)

	# if no adjacent square is dirty, move to a random square
	global directions
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
########################################################################################################################
																													Percept
########################################################################################################################
"""


def percept_no_memory(percept, map, agent):
	if (percept):
		return 'clean'

	return random.choice(directions)


visited = set()
agent_position = (0, 0)

visited = set()
agent_position = (0, 0)


def percept_memory(percept, world, agent):
	"""
	This agent has knowledge of the percept and memory of its past location.
	Args:
			percept: Whether the current square is dirty.
			world: Unused parameter.
			agent: Unused parameter.

	Returns:
			The next action (direction to move or "clean") for the robot.
	"""
	global visited, agent_position

	if percept:
		return 'clean'

	# Get the current location
	x, y = agent_position

	# Find the next unvisited square
	for direction in directions:
		next_x, next_y = calculate_next_position(direction, agent_position)
		if (next_x, next_y) not in visited and is_valid_move(world, (next_x, next_y)):
			# Update agent's position
			agent_position = (next_x, next_y)

			# Update visited locations
			visited.add(agent_position)

			return direction

	# If all adjacent squares are visited, choose a random direction
	next_move = random.choice(directions)
	next_x, next_y = calculate_next_position(next_move, agent_position)

	# Update agent's position
	agent_position = (next_x, next_y)

	# Update visited locations
	visited.add(agent_position)

	return next_move


visited = set()
agent_position = (0, 0)


def percept_memory(percept, world, agent):
	global visited, agent_position

	if percept:
		# If the agent perceives no dirt, it's a cleaning action
		return 'clean'

	# Get a random direction
	next_move = random.choice(directions)

	# Calculate the next position
	next_position = calculate_next_position(next_move)

	# Check if the next move revisits a visited location or is clean
	i = 0
	while next_position in visited:
		# If the next move is blocked (clean), choose another random direction
		next_move = random.choice(directions)
		next_position = calculate_next_position(next_move)
		i += 1
		if i > 100:
			next_move = random.choice(directions)
			break

	# Update agent's position
	agent_position = next_position

	# Update visited locations
	visited.add(agent_position)

	return next_move


def calculate_next_position(direction):
	x, y = agent_position
	if direction == 'north':
		y += 1
	elif direction == 'east':
		x += 1
	elif direction == 'south':
		y -= 1
	elif direction == 'west':
		x -= 1
	return (x, y)


"""
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                                        Agents
########################################################################################################################
                                                         Runs
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
"""

# input args for run: map_width, max_steps, agent_function, loss_function

# run(20, 50000, percept_memory, 'actions')
# run(20, 50000, entire_map_no_memory_actions, 'actions')
# run(20, 50000, entire_map_no_memory_actions, 'actions')
# run(20, 50000, entire_map_no_memory_dirt, 'dirt')

## input args for many_runs: map_width, max_steps, runs, agent_function, loss_function
#
# print(many_runs(20, 50000, 100, entire_map_no_memory_actions, 'actions'))
# print(many_runs(20, 50000, 100, entire_map_no_memory_dirt, 'actions'))
# print(many_runs(20, 5000, 100, entire_map_no_memory_actions, 'actions'))
# bareHeuristic = False
# print(many_runs(20, 5000, 100, entire_map_no_memory_actions, 'actions'))


# print(many_runs(20, 50000, 10, percept_memory, 'actions'))
# print(many_runs(20, 50000, 10, percept_no_memory, 'actions'))


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
			entire map No memory dirt :     entire_map_no_memory_actions
			entire map Memory actions :     entire_map_no_memory_actions
			entire map Memory dirt :        entire_map_no_memory_actions

			Neighbors No memory actions :   neighbors_no_memory_actions
			Neighbors No memory dirt :      neighbors_no_memory_actions
			Neighbors Memory actions :      neighbors_memory_actions
			Neighbors Memory dirt :         neighbors_memory_actions
			
			Percept No memory actions :     percept_no_memory
			Percept No memory dirt :        percept_no_memory
			Percept Memory actions :        percept_memory
			Percept Memory dirt :           percept_memory
	"""
	agents = [
		entire_map_no_memory_actions,  # entire map no memory
		entire_map_no_memory_actions,  # entire map    memory
		neighbors_no_memory_actions,  # neighbors  no memory
		neighbors_memory_actions,  # neighbors     memory
		percept_no_memory,  # percept    no memory
		percept_memory,  # percept       memory
	]
	cumulative_loss = 0

	# Run each agent and sum up their losses

	print(f'{"Agent": <35} {"Dirt Loss": <12}\t{"Actions Loss": <12} ')
	for agent in agents:
		loss_dirt = many_runs(20, 50000, 1000, agent, 'dirt')
		loss_actions = many_runs(20, 50000, 1000, agent, 'actions')

		print(f'{agent.__name__: <35}{loss_dirt: <12}\t{loss_actions: <12}')
		cumulative_loss += loss_dirt + loss_actions
	print(f'{"Total Loss": <35}{cumulative_loss}')

print("\n\n\n New Run\n\n\n")
run_all()
