import cv2
import numpy as np
import heapq
from typing import List
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

def euclidean_distance(a, b):
    return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

def get_successors(current, grid):
    successors = []
    x, y = current.x, current.y

    # Check all eight directions
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            if dx != 0 and dy != 0:
                if grid[x + dx][y] == 1 or grid[x][y + dy] == 1:
                    continue  # Diagonal movement not allowed if blocked by obstacles
            if 0 <= x + dx < len(grid) and 0 <= y + dy < len(grid[0]) and grid[x + dx][y + dy] != 1:
                successors.append(Node(x + dx, y + dy))
    return successors



def get_direction(current: Node, parent: Node):
    # Assuming get_direction function is defined based on your maze representation
    # This function should return the direction vector (dx, dy) from current to parent
    dx, dy = current.x - parent.x, current.y - parent.y
    if dx > 0:
        return (1, 0)
    elif dx < 0:
        return (-1, 0)
    elif dy > 0:
        return (0, 1)
    else:
        return (0, -1)


def is_valid_node(x, y, grid):
    # Assuming is_valid_node function is defined based on your maze representation
    # This function should check if (x, y) is within the grid boundaries and not an obstacle
    return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 255
def jump(current: Node, parent: Node, goal: Node, grid, visited):
    direction = get_direction(current, parent)  # Assuming get_direction function is defined
    while True:
        new_x, new_y = current.x + direction[0], current.y + direction[1]
        if not is_valid_node(new_x, new_y, grid) or (new_x, new_y) in visited:
            return current
        current = Node(new_x, new_y)
        if new_x == goal.x and new_y == goal.y:
            return current
        # Check for forced neighbors along the current direction
        if not is_valid_node(new_x + direction[0], new_y + direction[1], grid):
            return current

def jps(grid, start, goal):
    open_list = []
    open_set = set()
    closed_set = set()
    visited = set()  # Initialize the visited set for JPS

    heapq.heappush(open_list, start)
    open_set.add(start)

    while open_list:
        current = heapq.heappop(open_list)
        open_set.remove(current)
        closed_set.add(current)

        if current == goal:
            path = []
            while current is not None:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        for successor in get_successors(current, grid):
            jump_node = jump(successor, current, goal, grid, visited)  # Pass visited set to jump function
            if jump_node in closed_set:
                continue
            new_g = current.g + euclidean_distance(current, jump_node)
            if jump_node not in open_set or new_g < jump_node.g:
                jump_node.g = new_g
                jump_node.h = euclidean_distance(jump_node, goal)
                jump_node.f = jump_node.g + jump_node.h
                jump_node.parent = current
                heapq.heappush(open_list, jump_node)
                open_set.add(jump_node)
                visited.add(jump_node)  # Add jump_node to visited set

    return None



def mouse_event(event, x, y, flags, param):
    global start_point, end_point, selecting_start, selecting_end
    if event == cv2.EVENT_LBUTTONDOWN:
        if selecting_start:
            start_point = Node(y, x)
            print("Start point selected at:", (x, y))
            selecting_start = False
        elif selecting_end:
            end_point = Node(y, x)
            print("End point selected at:", (x, y))
            selecting_end = False

# Load the image
image = cv2.imread("map.PNG", cv2.IMREAD_GRAYSCALE)
original_image = cv2.imread("map.PNG")

# Threshold the image to get a binary representation of the maze
_, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)

# Create window and set mouse callback function
cv2.namedWindow("A* Visualization")
cv2.setMouseCallback("A* Visualization", mouse_event)

# Initialize start and end points
start_point = None
end_point = None
selecting_start = True
selecting_end = False

while True:
    if start_point and end_point:
    # Find the fastest path using JPS
        path = jps(binary_image, start_point, end_point)
        if path:
            # Draw the final path on the original image
            for x, y in path:
                cv2.circle(original_image, (y, x), 2, (0, 0, 255), -1)  # Mark the path in red
            cv2.circle(original_image, (start_point.y, start_point.x), 10, (0, 255, 0), -1)  # Mark the start point in green
            cv2.circle(original_image, (end_point.y, end_point.x), 10, (255, 0, 0), -1)  # Mark the end point in blue

            # Display the image with the path
            cv2.imshow("Maze with Path", original_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            break
        else:
            print("No path found!")
            break
    cv2.imshow("A* Visualization", image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        selecting_start = True
        selecting_end = False
    elif key == ord('e'):
        selecting_end = True
        selecting_start = False
    elif key == 27:
        break

cv2.destroyAllWindows()
