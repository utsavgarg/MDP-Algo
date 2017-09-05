import numpy as np

# DIRECTIONS
NORTH = 1
EAST = 2
SOUTH = 3
WEST = 4

# ACTIONS
LEFT = "A"
RIGHT = "D"
FORWARD = "W"

# MAP CONSTANTS
MAX_ROWS = 20
MAX_COLS = 15
START = np.asarray([18, 1])
GOAL = np.asarray([1, 13])
