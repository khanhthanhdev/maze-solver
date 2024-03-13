import cv2
import numpy as np
import heapq


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash((self.x, self.y))

def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

def astar(maze, start, end):
            open_list = []
            heapq.heappush(open_list, start)
            open_set = {start}
            closed_set = set()

            while open_list:
                current_node = heapq.heappop(open_list)
                open_set.remove(current_node)
                closed_set.add(current_node)

                if current_node == end:
                    path = []
                    current = current_node
                    while current is not None:
                        path.append((current.x, current.y))
                        current = current.parent
                    return path[::-1]

                neighbors = [(0, 10), (0, -10), (10, 0), (-10, 0)]
                for dx, dy in neighbors:
                    new_x, new_y = current_node.x + dx, current_node.y + dy
                    if 0 <= new_x < maze.shape[0] and 0 <= new_y < maze.shape[1] and maze[new_x][new_y] != 255:
                        neighbor = Node(new_x, new_y)
                        neighbor.g = current_node.g + 1
                        neighbor.h = heuristic(neighbor, end)
                        neighbor.f = neighbor.g + neighbor.h

                        if neighbor in closed_set:
                            continue

                        if neighbor not in open_set or neighbor.g < neighbor.g:
                            neighbor.parent = current_node
                            heapq.heappush(open_list, neighbor)
                            open_set.add(neighbor)

            return None

# Load the image
image = cv2.imread("map.PNG", cv2.IMREAD_GRAYSCALE)

# Threshold the image to get a binary representation of the maze
_, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY_INV)

# Initialize start and end points
start = Node(0, 0)
end = Node(binary_image.shape[0] - 1, binary_image.shape[1] - 1)

# Find the fastest path using A*
path = astar(binary_image, start, end)
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


        

