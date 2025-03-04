from collections import deque

def calculate_costs(input_file_path, output_file_path, target_x, target_y):
    """
    Calculates the cost for each cell in the matrix to the target cell using a wavefront algorithm.

    Args:
    - input_file_path (str): Path to the input text file containing the matrix.
    - output_file_path (str): Path to the output text file to save the cost matrix.
    - target_x (int): The x-coordinate of the target cell.
    - target_y (int): The y-coordinate of the target cell.
    """
    # Read the matrix from the input file
    with open(input_file_path, 'r') as file:
        labyrinth = [list(map(int, line.strip().split())) for line in file]

    rows = len(labyrinth)
    cols = len(labyrinth[0])

    # Print matrix dimensions and target coordinates for debugging
    print(f"Matrix dimensions: {rows} rows x {cols} cols")
    print(f"Target coordinates: ({target_x}, {target_y})")

    # Check if target coordinates are within matrix bounds
    if not (0 <= target_x < cols and 0 <= target_y < rows):
        raise ValueError("Target coordinates are out of matrix bounds")

    # Initialize distance matrix with '***'
    distance_matrix = [['***' for _ in range(cols)] for _ in range(rows)]

    # BFS initialization
    queue = deque([(target_x, target_y)])
    distance_matrix[target_y][target_x] = '001'  # Set the cost for the target cell to 1

    # Directions for 4-connectivity (up, down, left, right)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while queue:
        x, y = queue.popleft()
        current_distance = int(distance_matrix[y][x])  # Get current distance value

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if 0 <= nx < cols and 0 <= ny < rows and labyrinth[ny][nx] == 0:
                if distance_matrix[ny][nx] == '***':  # Only update if it's still '***'
                    distance_matrix[ny][nx] = f'{current_distance + 1:03}'  # Format with leading zeros
                    queue.append((nx, ny))

    # Write the cost matrix to the output file
    with open(output_file_path, 'w') as file:
        for row in distance_matrix:
            line = ' '.join(value for value in row)
            file.write(line + '\n')

    print(f"The cost matrix has been saved to {output_file_path}")

if __name__ == "__main__":
    # Example usage
    input_file_path = '/home/mru/discretized_map.txt'  # Path to the input file
    output_file_path = '/home/mru/cost_map.txt'  # Path to the output file
    target_x = 27  # Example target x-coordinate
    target_y = 107  # Example target y-coordinate

    calculate_costs(input_file_path, output_file_path, target_x, target_y)