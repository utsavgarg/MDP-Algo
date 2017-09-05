import numpy as np
import time

from Constants import NORTH, SOUTH, EAST, FORWARD, LEFT, RIGHT, START, MAX_ROWS, MAX_COLS


class Exploration:
    def __init__(self, realMap, timeLimit, sim=True):
        if sim:
            from Simulator import Robot
        self.timeLimit = timeLimit
        self.exploredArea = 0
        self.pathTaken = []
        self.currentMap = np.asarray([[0]*15]*20)
        self.robot = Robot(self.currentMap, NORTH, START, realMap)
        self.sensors = self.robot.getSensors()

    def getExploredArea(self):
        self.exploredArea = np.sum(self.currentMap != 0)

    def nextMove(self):
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
        directionFree = np.asarray([self.northFree(), self.eastFree(),
                                    self.southFree(), self.westFree()])
        dirs = np.asarray(['north', 'east', 'south', 'west'])
        dirs = dirs[order]
        directionFree = directionFree[order]
        if self.robot.direction == NORTH:
            print dirs[0], directionFree[0]
            return directionFree[0]
        elif self.robot.direction == EAST:
            print dirs[1], directionFree[1]
            return directionFree[1]
        elif self.robot.direction == SOUTH:
            print dirs[2], directionFree[2]
            return directionFree[2]
        else:
            print dirs[3], directionFree[3]
            return directionFree[3]

    def validMove(self, inds):
        for (r, c) in inds:
            if not ((0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS)):
                return False
        return (self.currentMap[inds[0][0], inds[0][1]] != 2 and
                self.currentMap[inds[1][0], inds[1][1]] != 2 and
                self.currentMap[inds[2][0], inds[2][1]] != 2)

    def northFree(self):
        r, c = self.robot.center
        inds = [[r-2, c], [r-2, c-1], [r-2, c+1]]
        return self.validMove(inds)

    def eastFree(self):
        r, c = self.robot.center
        inds = [[r, c+2], [r-1, c+2], [r+1, c+2]]
        return self.validMove(inds)

    def southFree(self):
        r, c = self.robot.center
        inds = [[r+2, c], [r+2, c-1], [r+2, c+1]]
        return self.validMove(inds)

    def westFree(self):
        r, c = self.robot.center
        inds = [[r, c-2], [r-1, c-2], [r+1, c-2]]
        return self.validMove(inds)

    def moveStep(self):
        self.robot.getSensors()
        self.nextMove()
        self.getExploredArea()
        if (self.exploredArea == 300):
            return True
        else:
            return False

    # Main method to perform exploration
    def explore(self):
        print "Starting exploration ..."
        startTime = time.time()
        endTime = startTime + self.timeLimit

        while(time.time() <= endTime and self.exploredArea <= 300):
            self.robot.getSensors()
            self.nextMove()
            self.getExploredArea()
            if (self.exploredArea == 300):
                break

        print self.currentMap
