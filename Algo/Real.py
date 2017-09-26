#!/usr/bin/env python
"""Implementation of the Robot class for simulation mode.
"""
import numpy as np

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

    def __init__(self, exploredMap, direction, start):
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
        self.setHead()
        self.movement = []
        self.markArea(start, 1)

    def markArea(self, center, value):
        """To mark a 3x3 neighbourhood around the center location with a particular
           value

        Args:
            center (list): Location to mark neighbourhood around
            value (int): The value to be filled
        """
        self.exploredMap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = value

    def setHead(self):
        if (self.direction == NORTH):
            self.head = self.center + [-1, 0]
        elif (self.direction == EAST):
            self.head = self.center + [0, 1]
        elif (self.direction == SOUTH):
            self.head = self.center + [1, 0]
        else:
            self.head = self.center + [0, -1]

    def getValue(self, inds, value, distance, sr):
        vals = []
        if (value >= distance):
            vals = [1]*distance
        else:
            val = round(value, -1)
            val = int(value//10)
            inds = inds[:val+1]
            vals = [1]*val + [2]
        for idx, (r, c) in enumerate(inds):
            if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS):
                if self.exploredMap[r][c] == 0:
                    self.exploredMap[r][c] = vals[idx]
                elif (sr):
                    self.exploredMap[r][c] = vals[idx]

    def getSensors(self, sensor_vals):
        """Generated indices to get values from sensors and gets the values using getValue() function.

        Returns:
            Numpy array of Numpy arrays: Sensor values from all sensors
        """
        distanceShort = 3
        distanceLong = 6
        r, c = self.center
        # sensor_vals = [FL_SR, FC_SR, FR_SR, RT_LR, RB_SR, LT_LR]

        # Front Left
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1],
                          sensor_vals[0], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[0], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip([r+1]*distanceShort, range(c-distanceShort, c))[::-1],
                          sensor_vals[0], distanceShort, True)
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort),
                          sensor_vals[0], distanceShort, True)

        # Front Center
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c]*distanceShort)[::-1],
                          sensor_vals[1], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip([r]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[1], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip([r]*distanceShort, range(c-distanceShort, c))[::-1],
                          sensor_vals[1], distanceShort, True)
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c]*distanceShort),
                          sensor_vals[1], distanceShort, True)

        # Front Right
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort)[::-1],
                          sensor_vals[2], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[2], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip([r-1]*distanceShort, range(c-distanceShort, c))[::-1],
                          sensor_vals[2], distanceShort, True)
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort),
                          sensor_vals[2], distanceShort, True)

        # Right Top
        if self.direction == NORTH:
            self.getValue(zip([r-1]*distanceLong, range(c+2, c+distanceLong+2)),
                          sensor_vals[3], distanceLong, False)
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceLong+2), [c+1]*distanceLong),
                          sensor_vals[3], distanceLong, False)
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceLong-1, r-1), [c-1]*distanceLong)[::-1],
                          sensor_vals[3], distanceLong, False)
        else:
            self.getValue(zip([r+1]*distanceLong, range(c-distanceLong, c))[::-1],
                          sensor_vals[3], distanceLong, False)

        # Right Bottom
        if self.direction == NORTH:
            self.getValue(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[4], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort),
                          sensor_vals[4], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort)[::-1],
                          sensor_vals[4], distanceShort, True)
        else:
            self.getValue(zip([r-1]*distanceShort, range(c-distanceShort, c))[::-1],
                          sensor_vals[4], distanceShort, True)

        # Left Top
        if self.direction == NORTH:
            self.getValue(zip([r-1]*distanceLong, range(c-distanceLong, c))[::-1],
                          sensor_vals[5], distanceLong, False)
        elif self.direction == EAST:
            self.getValue(zip(range(r-distanceLong-1, r-1), [c+1]*distanceLong)[::-1],
                          sensor_vals[5], distanceLong, False)
        elif self.direction == WEST:
            self.getValue(zip(range(r+2, r+distanceLong+2), [c-1]*distanceLong),
                          sensor_vals[5], distanceLong, False)
        else:
            self.getValue(zip([r+1]*distanceLong, range(c+2, c+distanceLong+2)),
                          sensor_vals[5], distanceLong, False)

    def moveBot(self, movement):
        """Simulates the bot movement based on current location, direction and received action

        Args:
            movement (string): Next movement received (see Constants)
        """
        self.movement.append(movement)
        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
            elif movement == LEFT:
                self.direction = WEST
            else:
                self.center = self.center + [-1, 0]
                self.markArea(self.center, 1)
            self.setHead()
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
            elif movement == LEFT:
                self.direction = NORTH
            else:
                self.center = self.center + [0, 1]
                self.markArea(self.center, 1)
            self.setHead()
        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
            elif movement == LEFT:
                self.direction = EAST
            else:
                self.center = self.center + [1, 0]
                self.markArea(self.center, 1)
            self.setHead()
        else:
            if movement == RIGHT:
                self.direction = NORTH
            elif movement == LEFT:
                self.direction = SOUTH
            else:
                self.center = self.center + [0, -1]
                self.markArea(self.center, 1)
            self.setHead()

    def descriptor_1(self):
        descriptor = np.zeros([20, 15]).astype(int)
        descriptor[self.exploredMap[::-1, :] != 0] = 1
        bits = '11'
        for row in descriptor:
            bits += ''.join(map(str, row.tolist()))
        bits += '11'
        hex_str = ['%X' % int(bits[i:i+4], 2) for i in range(0, len(bits)-3, 4)]
        return ''.join(hex_str)

    def descriptor_2(self):
        bits = ''
        for row in self.exploredMap[::-1, :]:
            for bit in row:
                if bit == 2:
                    bits += '1'
                elif bit != 0:
                    bits += '0'
        bits += '0'*(4 - len(bits) % 4)
        hex_str = ['%X' % int(bits[i:i+4], 2) for i in range(0, len(bits)-3, 4)]
        return ''.join(hex_str)
