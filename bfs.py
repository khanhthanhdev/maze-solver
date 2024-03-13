import cv2
import numpy as np
from collections import deque

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def bfs(maze, start, end, visualize=True):
    queue = deque()
    queue.append(start)
    visited = set()
    visited.add(start)

    while queue:
        current_node = queue.popleft()

        if current_node == end:
            path = []
            while current_node is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        if visualize:
            maze_with_path = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
            for node in visited:
                cv2.circle(maze_with_path, (node.y, node.x), 1, (0, 255, 0), -1)  # Draw visited nodes in green
            cv2.imshow("BFS Visualization", maze_with_path)
            cv2.waitKey(1)

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            new_x, new_y = current_node.x + dx, current_node.y + dy
            if 0 <= new_x < maze.shape[0] and 0 <= new_y < maze.shape[1] and maze[new_x][new_y] != 255:
                neighbor = Node(new_x, new_y)
                if neighbor not in visited:
                    neighbor.parent = current_node
                    queue.append(neighbor)
                    visited.add(neighbor)

    return None

def find_start_and_end(image):
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        start = Node(x, y)
        end = Node(x + w - 1, y + h - 1)
        return start, end
    else:
        return None, None

# Load the image
image = cv2.imread("map.PNG", cv2.IMREAD_GRAYSCALE)

# Threshold the image to get a binary representation of the maze
_, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY_INV)

# Initialize start and end points
start = None
end = None

# Find start and end points
start, end = find_start_and_end(binary_image)

# Check if start and end points are found
if start and end:
    # Find the path using BFS with visualization
    path = bfs(binary_image, start, end, visualize=True)
    if path:
        # Draw the final path on the original image
        maze_with_path = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for x, y in path:
            cv2.circle(maze_with_path, (y, x), 2, (0, 0, 255), -1)  # Mark the path in red
        cv2.circle(maze_with_path, (start.y, start.x), 2, (0, 255, 0), -1)  # Mark the start point in green
        cv2.circle(maze_with_path, (end.y, end.x), 2, (255, 0, 0), -1)  # Mark the end point in blue

        # Display the image with the path
        cv2.imshow("Maze with Path", maze_with_path)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No path found!")
else:
    print("Start and end points not found.")
