import cv2
import numpy as np

def discretize_map(image_path, robot_size):
    """
    Discretizes a map image into a matrix M with 0 representing free space
    and 1 representing obstacles.

    Args:
    - image_path (str): Path to the binary map image (PNG format).
    - robot_size (int): Size of the robot in pixels (used to define grid cell size).

    Returns:
    - M (np.ndarray): Discretized map as a 2D numpy array.
    """
    # Load the image as a grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if image is None:
        raise FileNotFoundError(f"Image file not found at {image_path}. Please check the path.")

    # Convert image to binary (thresholding)
    _, binary_map = cv2.threshold(image, 127, 1, cv2.THRESH_BINARY_INV)

    # Determine the dimensions of the grid
    grid_size_x = image.shape[1] // robot_size
    grid_size_y = image.shape[0] // robot_size

    # Initialize the discretized map matrix
    M = np.zeros((grid_size_y, grid_size_x), dtype=int)

    # Fill in the matrix M with 0 for free space and 1 for obstacles
    for i in range(grid_size_y):
        for j in range(grid_size_x):
            cell = binary_map[i*robot_size:(i+1)*robot_size, j*robot_size:(j+1)*robot_size]
            if np.any(cell == 1):
                M[i, j] = 1

    return M

if __name__ == "__main__":
    image_path = "/home/mru/task_ws/src/robotcraft_maze/world/540x540-90cm.png"
    robot_size = 5  # Example robot size in pixels
    M = discretize_map(image_path, robot_size)

    # Change print options to display the full matrix
    np.set_printoptions(threshold=np.inf)

    # Print the discretized map matrix
    print(M)

    # Save the matrix to a text file
    output_file_path = "/home/mru/discretized_map.txt"
    np.savetxt(output_file_path, M, fmt='%d')

    print(f"Discretized map has been saved to {output_file_path}.")