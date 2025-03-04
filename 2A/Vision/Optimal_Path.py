import numpy as np
import heapq

def a_star(cost_matrix, start_x, start_y, target_x, target_y):
    """
    Finds the shortest path from the starting point to the target using the A* algorithm.

    Args:
    - cost_matrix (np.ndarray): 2D array containing the cost values.
    - start_x (int): The x-coordinate of the starting point.
    - start_y (int): The y-coordinate of the starting point.
    - target_x (int): The x-coordinate of the target point.
    - target_y (int): The y-coordinate of the target point.

    Returns:
    - path (list of tuples): List of (x, y) coordinates representing the path from start to target.
    """
    rows, cols = cost_matrix.shape
    start = (start_x, start_y)
    target = (target_x, target_y)

    # Priority queue for A* (min-heap)
    open_list = []
    heapq.heappush(open_list, (0, start))

    # Dictionaries for costs and paths
    g_costs = {start: 0}
    f_costs = {start: heuristic(start, target)}
    came_from = {}

    # Directions for movement: right, left, down, up
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current == target:
            return reconstruct_path(came_from, current)

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if (0 <= neighbor[0] < cols) and (0 <= neighbor[1] < rows):
                new_g_cost = g_costs[current] + cost_matrix[neighbor[1]][neighbor[0]]

                if neighbor not in g_costs or new_g_cost < g_costs[neighbor]:
                    g_costs[neighbor] = new_g_cost
                    f_cost = new_g_cost + heuristic(neighbor, target)
                    f_costs[neighbor] = f_cost
                    heapq.heappush(open_list, (f_cost, neighbor))
                    came_from[neighbor] = current

    return []

def heuristic(node, target):
    """
    Heuristic function for A* (Manhattan distance).

    Args:
    - node (tuple): Current position (x, y).
    - target (tuple): Target position (x, y).

    Returns:
    - h_cost (float): Heuristic cost (Manhattan distance).
    """
    return abs(node[0] - target[0]) + abs(node[1] - target[1])

def reconstruct_path(came_from, current):
    """
    Reconstructs the path from the target to the start.

    Args:
    - came_from (dict): Dictionary mapping nodes to their predecessors.
    - current (tuple): The ending node.

    Returns:
    - path (list of tuples): List of (x, y) coordinates representing the path.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def create_path_map(cost_matrix, path, filename='path_map.txt'):
    """
    Creates a new map where path cells are marked with '+++' instead of their original values.

    Args:
    - cost_matrix (np.ndarray): 2D array containing the cost values.
    - path (list of tuples): List of (x, y) coordinates representing the path.
    - filename (str): Filename to save the new map.
    """
    path_map = np.char.array(cost_matrix.astype(str))

    # Format cost matrix to ensure three digits and replace np.inf with '***'
    for (y, x), value in np.ndenumerate(cost_matrix):
        if value == np.inf:
            path_map[y, x] = '***'
        else:
            path_map[y, x] = f'{int(value):03}'

    # Mark path cells with '+++'
    for (x, y) in path:
        path_map[y, x] = '   '

    # Save the new map to a file
    np.savetxt(filename, path_map, fmt='%s')

if __name__ == "__main__":
    # Load the cost matrix from a file
    cost_matrix = np.loadtxt('/home/mru/cost_map.txt', dtype=str)

    # Convert cost matrix elements to integers where applicable
    cost_matrix = np.array([[int(x) if x != '***' else np.inf for x in row] for row in cost_matrix])

    # Define start and target positions (example)
    start_x = 0  # x-coordinate of the start
    start_y = 11  # y-coordinate of the start
    target_x = 27  # x-coordinate of the target (example)
    target_y = 107  # y-coordinate of the target (example)

    # Find the path from the start to the target
    path = a_star(cost_matrix, start_x, start_y, target_x, target_y)

    # Specify the path for the new map
    path_to_save = '/home/mru/path_map.txt'

    # Create and save the path map
    create_path_map(cost_matrix, path, path_to_save)

    print("Path from start to target:", path)
    print(f"Path map saved as '{path_to_save}'")