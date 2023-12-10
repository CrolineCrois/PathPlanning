import cv2
import sys
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from pyqtgraph.graphicsItems.ImageItem import ImageItem

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap, QColor
from PyQt6.QtWidgets import QApplication, QGraphicsScene, QGraphicsView, QGraphicsPixmapItem, QGraphicsRectItem, QMainWindow

import numpy as np

# A* Path Planning
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start node to current node
        self.h = 0  # Heuristic cost from current node to target node
        self.f = 0  # Total cost (g + h)

def heuristic(a, b):
    x1, y1 = a
    x2, y2 = b
    return abs(x1 - x2) + abs(y1 - y2)  # Manhattan distance

def mark_path_in_field(field_map, path):
    for x, y in path:
        if 0 <= x < pos_array_width and 0 <= y < pos_array_height:
            field_map[x][y] = 2  # Set the path positions to 2 (or any value you prefer)
    return field_map

def astar(field_map, start, end):
    open_set = []
    closed_set = set()

    start_node = Node(start)
    end_node = Node(end)

    open_set.append(start_node)

    while open_set:
        # print(open_set)
        # print(closed_set)
        current_node = min(open_set, key=lambda node: node.f)
        open_set.remove(current_node)
        closed_set.add(current_node.position)

        if current_node.position == end_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]
        
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]  # Adjacent cells
        for neighbor in neighbors:
            new_position = (current_node.position[0] + neighbor[0], current_node.position[1] + neighbor[1])

            if (
                new_position[0] >= 0
                and new_position[0] < pos_array_width
                and new_position[1] >= 0
                and new_position[1] < pos_array_height
                and field_map[new_position[0]][new_position[1]] == 0
            ):
                if new_position in closed_set:
                    continue

                new_node = Node(new_position, current_node)
                new_node.g = current_node.g + 1  # Assuming each move costs 1
                new_node.h = heuristic(new_node.position, end_node.position)
                new_node.f = new_node.g + new_node.h

                if any(node.position == new_node.position and node.g > new_node.g for node in open_set):
                    continue

                open_set.append(new_node)

    return []  # No path found

def mark_rectangle_obstacle(field_map, start, end):
    x1, y1 = start
    x2, y2 = end

    for x in range(min(x1, x2), max(x1, x2) + 1):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            
            if (
                x >= 0
                and x < pos_array_width
                and y >= 0
                and y < pos_array_height
            ):
                field_map[x][y] = 1  # Mark this position as an obstacle (1)

def create_field_image(field_map):
    field_image = field_map.T  # Transpose the field_map for correct display
    return field_image            

# Constants
field_width_feet = 10  # the width of the field in feet
field_height_feet = 10  # the height of the field in feet

field_width = field_width_feet * 12  # the width of the field in inches
field_height = field_height_feet * 12  # the height of the field in inches
position_resolution = 0.5  # the minimum interval of position in inches

pos_array_width = int(field_width / position_resolution)
pos_array_height = int(field_height / position_resolution)

# create an array of zeros in the size of our field
# to keep our data light, we'll say that zero means that the ground is obstacle-free/traversable
field_map = np.zeros((pos_array_width, pos_array_height))
# field_map = cv2.imread('/Users/rishayjain/GRT_DS/.venv/map.png', cv2.IMREAD_GRAYSCALE)
# field_map = cv2.resize(field_map, (513, 250), interpolation=cv2.INTER_LINEAR)

current_pos = (int(pos_array_width/6), int(pos_array_height/6))
desired_pos = (pos_array_width - 1, pos_array_height - 1)
# desired_pos = (pos_array_width - 1, int(pos_array_height/2))

mark_rectangle_obstacle(field_map, (int(pos_array_width/4), int(pos_array_height/3)), (int((2 * pos_array_width)/4), int((2 * pos_array_height)/3)))
mark_rectangle_obstacle(field_map, (int(pos_array_width/4), int(pos_array_height/4)), (int((2 * pos_array_width)/5), int((2 * pos_array_height)/4)))
mark_rectangle_obstacle(field_map, (int(pos_array_width/7), int(pos_array_height/4)), (int((2 * pos_array_width)/2), int((2 * pos_array_height)/6)))

path = astar(field_map, current_pos, desired_pos)
field_map = mark_path_in_field(field_map, path)
field_map[0][0] = -1
field_map[pos_array_width - 1][pos_array_height - 1] = -2

class Path_Window(QMainWindow):
    
    def __init__(self, parent=None):
        super(Path_Window, self).__init__(parent)
        self.setWindowTitle("Path Viewer") # set title of application window
        
        image = pg.ImageView()
        image.setImage(field_map)
        
        self.setCentralWidget(image)
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    player = Path_Window()
    player.resize(640, 480)
    player.show()
    sys.exit(app.exec())