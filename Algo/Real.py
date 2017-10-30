#!/usr/bin/env python
"""Implementation of the Robot class for simulation mode.
"""
import numpy as np

from Constants import MAX_ROWS, MAX_COLS, NORTH, SOUTH, EAST, WEST, RIGHT, LEFT, BOTTOM_LEFT_CORNER, BOTTOM_RIGHT_CORNER, TOP_RIGHT_CORNER, TOP_LEFT_CORNER

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
        self.marked = np.zeros((20, 15))
        self.setHead()
        self.movement = []
        self.markArea(start, 1)
        self.stepCounter = 0
        self.phase = 1

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

    def getValue(self, inds, value, distance, sr, right=False):
        if value != 0:
            value = round(value - 6, -1)
        vals = []
        if (value >= distance*10):
            vals = [1]*distance
        else:
            value = int(value//10)
            inds = inds[:value+1]
            vals = [1]*value + [2]
        for idx, (r, c) in enumerate(inds):
            if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS):
                # for override
                if self.phase == 1:
                    if (self.exploredMap[r][c] == 2 and vals[idx] == 1 and
                       self.marked[r][c] < 2 and sr and (not right)):
                        self.exploredMap[r][c] = vals[idx]
                        self.marked[r][c] == 1
                    elif self.exploredMap[r][c] == 2:
                        break
                    elif (self.exploredMap[r][c] == 0):
                        self.exploredMap[r][c] = vals[idx]
                    self.marked[r][c] += 1
                else:
                    if self.exploredMap[r][c] == 2:
                        break
                    elif (self.exploredMap[r][c] == 0):
                        self.exploredMap[r][c] = vals[idx]
                # without override
                # if self.exploredMap[r][c] == 0:
                #     self.exploredMap[r][c] = vals[idx]
                # # elif (sr and self.marked[r][c] == 0):
                # #     self.exploredMap[r][c] = vals[idx]
                # #     self.marked[r][c] = 1
                # elif self.exploredMap[r][c] == 2:
                #     break


    def getSensors(self, sensor_vals):
        """Generated indices to get values from sensors and gets the values using getValue() function.

        Returns:
            Numpy array of Numpy arrays: Sensor values from all sensors
        """
        distanceShort = 3
        distanceLong = 5
        r, c = self.center
        # sensor_vals = [FL_SR, FC_SR, FR_SR, RT_SR, RB_LR, LT_LR]

        # Front Left
        if self.direction == NORTH:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1],
                          sensor_vals[0], distanceShort, True)
        elif self.direction == EAST:
            self.getValue(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[0], distanceShort, True)
        elif self.direction == WEST:
            self.getValue(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
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
            self.getValue(zip([r]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
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
            self.getValue(zip([r-1]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
                          sensor_vals[2], distanceShort, True)
        else:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort),
                          sensor_vals[2], distanceShort, True)

        # Right Top
        if self.direction == NORTH:
            self.getValue(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2)),
                          sensor_vals[3], distanceShort, True, True)
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort),
                          sensor_vals[3], distanceShort, True, True)
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)[::-1],
                          sensor_vals[3], distanceShort, True, True)
        else:
            self.getValue(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1))[::-1],
                          sensor_vals[3], distanceShort, True, True)

        # Right Bottom
        if self.direction == NORTH:
            self.getValue(zip([r+1]*distanceLong, range(c+2, c+distanceLong+2)),
                          sensor_vals[4], distanceLong, False)
        elif self.direction == EAST:
            self.getValue(zip(range(r+2, r+distanceLong+2), [c-1]*distanceLong),
                          sensor_vals[4], distanceLong, False)
        elif self.direction == WEST:
            self.getValue(zip(range(r-distanceLong-1, r-1), [c+1]*distanceLong)[::-1],
                          sensor_vals[4], distanceLong, False)
        else:
            self.getValue(zip([r-1]*distanceLong, range(c-distanceLong-1, c-1))[::-1],
                          sensor_vals[4], distanceLong, False)

        # Left Top
        if self.direction == NORTH:
            self.getValue(zip([r-1]*distanceLong, range(c-distanceLong-1, c-1))[::-1],
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

    def is_corner(self):
        r, c = self.center
        if self.direction == NORTH:
            fw = False
            fw = (r-2 == -1) or (r-2 != -1 and self.exploredMap[r-2][c-1] == 2 and
                                 self.exploredMap[r-2][c] == 2
                                 and self.exploredMap[r-2][c+1] == 2)
            rw = (c+2 == MAX_COLS) or (c+2 != MAX_COLS and self.exploredMap[r-1][c+2] == 2 and
                                       self.exploredMap[r][c+2] == 2
                                       and self.exploredMap[r+1][c+2] == 2)
            if fw and rw:
                return True
            else:
                return False
        elif self.direction == EAST:
            fw = (c+2 == MAX_COLS) or (c+2 != MAX_COLS and self.exploredMap[r-1][c+2] == 2 and
                                       self.exploredMap[r][c+2] == 2
                                       and self.exploredMap[r+1][c+2] == 2)
            rw = (r+2 == MAX_ROWS) or (r+2 != MAX_ROWS and self.exploredMap[r+2][c-1] == 2 and
                                       self.exploredMap[r+2][c] == 2
                                       and self.exploredMap[r+2][c+1] == 2)
            if fw and rw:
                return True
            else:
                return False
        elif self.direction == SOUTH:
            fw = (r+2 == MAX_ROWS) or (r+2 != MAX_ROWS and self.exploredMap[r+2][c-1] == 2 and
                                       self.exploredMap[r+2][c] == 2
                                       and self.exploredMap[r+2][c+1] == 2)
            rw = (c-2 == -1) or (c-2 != -1 and self.exploredMap[r-1][c-2] == 2 and
                                 self.exploredMap[r][c-2] == 2
                                 and self.exploredMap[r+1][c-2] == 2)
            if fw and rw:
                return True
            else:
                return False
        else:
            fw = (c-2 == -1) or (c-2 != -1 and self.exploredMap[r-1][c-2] == 2 and
                                 self.exploredMap[r][c-2] == 2
                                 and self.exploredMap[r+1][c-2] == 2)
            rw = (r-2 == -1) or (r-2 != -1 and self.exploredMap[r-2][c-1] == 2 and
                                 self.exploredMap[r-2][c] == 2
                                 and self.exploredMap[r-2][c+1] == 2)
            if fw and rw:
                return True
            else:
                return False

    def can_calibrate_front(self):
        r, c = self.center
        flag = [False, None]
        if self.direction == NORTH:
            for i in range(2, 3):
                if ((r-i) < 0):
                    flag = [True, 'C']
                    break
                elif ((r - i) >= 0 and (self.exploredMap[r-i][c-1] == 2 and
                      self.exploredMap[r-i][c] == 2 and self.exploredMap[r-i][c+1] == 2)):
                    flag = [True, 'C']
                    break
        elif self.direction == WEST:
            for i in range(2, 3):
                if ((c-i) < 0):
                    flag = [True, 'C']
                    break
                elif ((c-i) >= 0 and (self.exploredMap[r-1][c-i] == 2 and
                      self.exploredMap[r][c-i] == 2 and self.exploredMap[r+1][c-i] == 2)):
                    flag = [True, 'C']
                    break
        elif self.direction == EAST:
            for i in range(2, 3):
                if ((c + i) == MAX_COLS):
                    flag = [True, 'C']
                    break
                elif ((c + i) < MAX_COLS and (self.exploredMap[r-1][c+i] == 2 and
                      self.exploredMap[r][c+i] == 2 and self.exploredMap[r+1][c+i] == 2)):
                    flag = [True, 'C']
                    break
        else:
            for i in range(2, 3):
                if ((r+i) == MAX_ROWS):
                    flag = [True, 'C']
                    break
                elif ((r+i) < MAX_ROWS and (self.exploredMap[r+i][c-1] == 2 and
                      self.exploredMap[r+i][c] == 2 and self.exploredMap[r+i][c+1] == 2)):
                    flag = [True, 'C']
                    break
        return flag

    def can_calibrate_right(self):
        r, c = self.center
        flag = [False, None]
        if self.direction == NORTH:
            for i in range(2, 3):
                if ((c + i) == MAX_COLS):
                    flag = [True, 'R']
                    break
                elif ((c + i) < MAX_COLS and (self.exploredMap[r-1, c+i] == 2 and
                      self.exploredMap[r+1, c+i] == 2)):
                    flag = [True, 'R']
                    break
        elif self.direction == WEST:
            for i in range(2, 3):
                if ((r - i) < 0):
                    flag = [True, 'R']
                    break
                elif ((r - i) >= 0 and (self.exploredMap[r-i, c-1] == 2 and
                      self.exploredMap[r-i, c+1] == 2)):
                    flag = [True, 'R']
                    break
        elif self.direction == EAST:
            for i in range(2, 3):
                if ((r + i) == MAX_ROWS):
                    flag = [True, 'R']
                    break
                elif ((r + i) < MAX_ROWS and (self.exploredMap[r+i, c-1] == 2 and
                      self.exploredMap[r+i, c+1] == 2)):
                    flag = [True, 'R']
                    break
        else:
            for i in range(2, 3):
                if ((c-i) < 0):
                    flag = [True, 'R']
                    break
                elif ((c - i) >= 0 and (self.exploredMap[r-1, c-i] == 2 and
                      self.exploredMap[r+1, c-i] == 2)):
                    flag = [True, 'R']
                    break
        return flag

    def moveBot(self, movement):
        """Simulates the bot movement based on current location, direction and received action

        Args:
            movement (string): Next movement received (see Constants)
        """
        self.stepCounter += 1
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
