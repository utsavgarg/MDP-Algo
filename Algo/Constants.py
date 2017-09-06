#!/usr/bin/env python
"""Predefined constant values to be used in the algorithms

"""

import numpy as np

__author__ = "Utsav Garg"

# DIRECTIONS
NORTH = 1
EAST = 2
SOUTH = 3
WEST = 4

# MOVEMENTS
LEFT = "A"
RIGHT = "D"
FORWARD = "W"

# MAP CONSTANTS
MAX_ROWS = 20
MAX_COLS = 15
START = np.asarray([18, 1])
GOAL = np.asarray([1, 13])
