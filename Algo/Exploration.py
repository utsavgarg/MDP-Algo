#!/usr/bin/env python
"""Implementation of exploration algorithm for maze solving robot

"""
import numpy as np
import time

from Constants import NORTH, SOUTH, EAST, FORWARD, LEFT, RIGHT, START, MAX_ROWS, MAX_COLS

__author__ = "Utsav Garg"


class Exploration:
    """Implementation of the Right-Wall hugging algorithm for a maze solving
       robot.
       The implementation assumes that the robot starts at the bottom-left corner of the map,
       i.e. (Rows - 2, 1). And the robot is facing North


    Attributes:
        currentMap (Numpy array): To store the current state of the exploration map
        exploredArea (int): Count of the number of cells explored
        robot (Robot): Instance of the Robot class
        sensors (list of Numpy arrays): Readings from all sensors
        timeLimit (int): Maximum time allowed for exploration
    """

    def __init__(self, realMap, timeLimit, sim=True):
        """Constructor to initialise an instance of the Exploration class

        Args:
            realMap (string): File name for real map during simulation stage
            timeLimit (int): Maximum time allowed for exploration
            sim (bool, optional): To specify is the exploration mode is simulation or real
        """
        if sim:
            from Simulator import Robot
        self.timeLimit = timeLimit
        self.exploredArea = 0
        self.currentMap = np.asarray([[0]*15]*20)
        self.robot = Robot(self.currentMap, NORTH, START, realMap)
        self.sensors = self.robot.getSensors()

    def getExploredArea(self):
        """Updates the total number of cells explored at the current state
        """
        self.exploredArea = np.sum(self.currentMap != 0)

    def nextMove(self):
        """Decides which direction is free and commands the robot the next action
        """
        if (self.checkFree([1, 2, 3, 0])):
            self.robot.moveBot(RIGHT)
            if (self.checkFree([0, 1, 2, 3])):
                self.robot.moveBot(FORWARD)
        elif (self.checkFree([0, 1, 2, 3])):
            self.robot.moveBot(FORWARD)
        elif (self.checkFree([3, 0, 1, 2])):
            self.robot.moveBot(LEFT)
            if (self.checkFree([0, 1, 2, 3])):
                self.robot.moveBot(FORWARD)
        else:
            self.robot.moveBot(RIGHT)
            self.robot.moveBot(RIGHT)

    def checkFree(self, order):
        """Checks if a specific direction is free to move to

        Args:
            order (list): Ordering for the directionFree list based on the
                          the next move (Right, Left, Forward)

        Returns:
            bool: If the queried direction is free
        """
        directionFree = np.asarray([self.northFree(), self.eastFree(),
                                    self.southFree(), self.westFree()])
        directionFree = directionFree[order]
        if self.robot.direction == NORTH:
            return directionFree[0]
        elif self.robot.direction == EAST:
            return directionFree[1]
        elif self.robot.direction == SOUTH:
            return directionFree[2]
        else:
            return directionFree[3]

    def validMove(self, inds):
        """Checks if all the three cells on one side of the robot are free

        Args:
            inds (list of list): List of cell indices to be checked

        Returns:
            bool: If all indices are free (no obstacle)
        """
        for (r, c) in inds:
            if not ((0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS)):
                return False
        return (self.currentMap[inds[0][0], inds[0][1]] != 2 and
                self.currentMap[inds[1][0], inds[1][1]] != 2 and
                self.currentMap[inds[2][0], inds[2][1]] != 2)

    def northFree(self):
        """Checks if the north direction is free to move

        Returns:
            bool: if north is free
        """
        r, c = self.robot.center
        inds = [[r-2, c], [r-2, c-1], [r-2, c+1]]
        return self.validMove(inds)

    def eastFree(self):
        """Checks if the east direction is free to move

        Returns:
            bool: if east is free
        """
        r, c = self.robot.center
        inds = [[r, c+2], [r-1, c+2], [r+1, c+2]]
        return self.validMove(inds)

    def southFree(self):
        """Checks if the south direction is free to move

        Returns:
            bool: if south is free
        """
        r, c = self.robot.center
        inds = [[r+2, c], [r+2, c-1], [r+2, c+1]]
        return self.validMove(inds)

    def westFree(self):
        """Checks if the west direction is free to move

        Returns:
            bool: if west is free
        """
        r, c = self.robot.center
        inds = [[r, c-2], [r-1, c-2], [r+1, c-2]]
        return self.validMove(inds)

    def moveStep(self):
        """Moves the robot one step for exploration

        Returns:
            bool: True is the map is fully explored
        """
        self.robot.getSensors()
        self.nextMove()
        self.getExploredArea()
        if (self.exploredArea == 300):
            return True
        else:
            return False

    def explore(self):
        """Runs the exploration till the map is fully explored of time runs out
        """
        print "Starting exploration ..."
        startTime = time.time()
        endTime = startTime + self.timeLimit

        while(time.time() <= endTime):
            if (self.moveStep()):
                print "Exploration completed !"
                print self.currentMap
                return

        print "Time over !"
        print self.currentMap
        return
