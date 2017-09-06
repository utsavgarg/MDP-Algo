#!/usr/bin/env python
"""Implementation of the Robot class for simulation mode.
"""
import numpy as np
import os

from Constants import MAX_ROWS, MAX_COLS, NORTH, SOUTH, EAST, WEST, RIGHT, LEFT

__author__ = "Utsav Garg"


class Robot:

    """Robot class keeps track of the current location and direction of the robot,
       gets values from the distance sensors and sends commands to move the robot

    Attributes:
        center (list): Center location of the robot
        direction (int): Current direction of the robot (see Constants)
        exploredMap (Numpy array): The current explored map
        head (list): Location of the head of the robot
        map (Numpy array): Real map of the arena for simulation mode
        movement (string): The latest movement of the robot (see Constants)
        realMap (string): File name for map
    """

    def __init__(self, exploredMap, direction, start, realMap):
        """Constructor to initialise an instance of the Exploration class

        Args:
            exploredMap (Numpy array): To initial state of the exploration map
            direction (int): The starting direction for the robot (see Constants)
            start (list): The starting center location of the robot
            realMap (string): File name of the real map
        """
        self.exploredMap = exploredMap
        self.direction = direction
        self.center = np.asarray(start)
        self.head = np.asarray([start[0]-1, start[1]])
        self.realMap = realMap
        self.movement = []
        self.map = self.loadMap()
        self.markArea(start, 1)

    def markArea(self, center, value):
        """To mark a 3x3 neighbourhood around the center location with a particular
           value

        Args:
            center (list): Location to mark neighbourhood around
            value (int): The value to be filled
        """
        self.exploredMap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = value

    def loadMap(self):
        """To load the real map file and store it as a Numpy array

        Returns:
            Numpy array: Real map
        """
        with open(os.path.join('Maps', self.realMap)) as f:
            return np.genfromtxt(f, dtype=int, delimiter=1)

    def getValue(self, inds):
        """To get sensor values (from the real map for simulation) at the given indices

        Args:
            inds (list of list): Indices to get sensor values at

        Returns:
            list: values of the sensors and puts none after the first obstacle is encountered
        """
        vals = []
        for (r, c) in inds:
            if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS):
                if self.map[r][c] == 2:
                    if self.exploredMap[r][c] == 0:
                        self.exploredMap[r][c] = 2
                    vals.append(2)
                    break
                else:
                    if self.exploredMap[r][c] == 0:
                        self.exploredMap[r][c] = 1
                    vals.append(1)
        if len(vals) < len(inds):
            vals = vals + [None]*(len(inds)-len(vals))
        return vals

    def getSensors(self):
        """Generated indices to get values from sensors and gets the values using getValue() function.
           For this simulator it is assumed that the sensors can get values up a distance of 4 cells

        Returns:
            Numpy array of Numpy arrays: Sensor values from all sensors
        """
        sensors = np.asarray([[None]*4]*6)
        maxDistance = 4
        r, c = self.center

        # Front Left
        if self.direction == NORTH:
            sensors[0] = self.getValue(zip(range(r-maxDistance-1, r-1), [c-1]*maxDistance)[::-1])
        elif self.direction == EAST:
            sensors[0] = self.getValue(zip([r-1]*maxDistance, range(c+2, c+maxDistance+2)))
        elif self.direction == WEST:
            sensors[0] = self.getValue(zip([r+1]*maxDistance, range(c-maxDistance, c))[::-1])
        else:
            sensors[0] = self.getValue(zip(range(r+2, r+maxDistance+2), [c+1]*maxDistance))

        # Front Center
        if self.direction == NORTH:
            sensors[1] = self.getValue(zip(range(r-maxDistance-1, r-1), [c]*maxDistance)[::-1])
        elif self.direction == EAST:
            sensors[1] = self.getValue(zip([r]*maxDistance, range(c+2, c+maxDistance+2)))
        elif self.direction == WEST:
            sensors[1] = self.getValue(zip([r]*maxDistance, range(c-maxDistance, c))[::-1])
        else:
            sensors[1] = self.getValue(zip(range(r+2, r+maxDistance+2), [c]*maxDistance))

        # Front Right
        if self.direction == NORTH:
            sensors[2] = self.getValue(zip(range(r-maxDistance-1, r-1), [c+1]*maxDistance)[::-1])
        elif self.direction == EAST:
            sensors[2] = self.getValue(zip([r+1]*maxDistance, range(c+2, c+maxDistance+2)))
        elif self.direction == WEST:
            sensors[2] = self.getValue(zip([r-1]*maxDistance, range(c-maxDistance, c))[::-1])
        else:
            sensors[2] = self.getValue(zip(range(r+2, r+maxDistance+2), [c-1]*maxDistance))

        # Right Top
        if self.direction == NORTH:
            sensors[3] = self.getValue(zip([r-1]*maxDistance, range(c+2, c+maxDistance+2)))
        elif self.direction == EAST:
            sensors[3] = self.getValue(zip(range(r+2, r+maxDistance+2), [c+1]*maxDistance))
        elif self.direction == WEST:
            sensors[3] = self.getValue(zip(range(r-maxDistance-1, r-1), [c-1]*maxDistance)[::-1])
        else:
            sensors[3] = self.getValue(zip([r+1]*maxDistance, range(c-maxDistance, c))[::-1])

        # Right Bottom
        if self.direction == NORTH:
            sensors[4] = self.getValue(zip([r+1]*maxDistance, range(c+2, c+maxDistance+2)))
        elif self.direction == EAST:
            sensors[4] = self.getValue(zip(range(r+2, r+maxDistance+2), [c-1]*maxDistance))
        elif self.direction == WEST:
            sensors[4] = self.getValue(zip(range(r-maxDistance-1, r-1), [c+1]*maxDistance)[::-1])
        else:
            sensors[4] = self.getValue(zip([r-1]*maxDistance, range(c-maxDistance, c))[::-1])

        # Left Top
        if self.direction == NORTH:
            sensors[5] = self.getValue(zip([r-1]*maxDistance, range(c-maxDistance, c))[::-1])
        elif self.direction == EAST:
            sensors[5] = self.getValue(zip(range(r-maxDistance-1, r-1), [c+1]*maxDistance)[::-1])
        elif self.direction == WEST:
            sensors[5] = self.getValue(zip(range(r+2, r+maxDistance+2), [c-1]*maxDistance))
        else:
            sensors[5] = self.getValue(zip([r+1]*maxDistance, range(c+2, c+maxDistance+2)))

        return sensors

    def moveBot(self, movement):
        """Simulates the bot movement based on current location, direction and received action

        Args:
            movement (string): Next movement received (see Constants)
        """
        self.movement.append(movement)
        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
                self.head = self.head + [1, 1]
            elif movement == LEFT:
                self.direction = WEST
                self.head = self.head + [1, -1]
            else:
                self.head = self.head + [-1, 0]
                self.center = self.center + [-1, 0]
                self.markArea(self.center, 1)
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
                self.head = self.head + [1, -1]
            elif movement == LEFT:
                self.direction = NORTH
                self.head = self.head + [-1, -1]
            else:
                self.head = self.head + [0, 1]
                self.center = self.center + [0, 1]
                self.markArea(self.center, 1)
        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
                self.head = self.head + [-1, -1]
            elif movement == LEFT:
                self.direction = EAST
                self.head = self.head + [-1, 1]
            else:
                self.head = self.head + [1, 0]
                self.center = self.center + [1, 0]
                self.markArea(self.center, 1)
        else:
            if movement == RIGHT:
                self.direction = NORTH
                self.head = self.head + [-1, 1]
            elif movement == LEFT:
                self.direction = SOUTH
                self.head = self.head + [1, 1]
            else:
                self.head = self.head + [0, -1]
                self.center = self.center + [0, -1]
                self.markArea(self.center, 1)
