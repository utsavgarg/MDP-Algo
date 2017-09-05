import numpy as np
import os

from Constants import MAX_ROWS, MAX_COLS, NORTH, SOUTH, EAST, WEST, RIGHT, LEFT


class Robot:
    def __init__(self, exploredMap, direction, start, realMap):
        self.exploredMap = exploredMap
        self.direction = direction
        self.center = np.asarray(start)
        self.head = np.asarray([start[0]-1, start[1]])
        self.realMap = realMap
        self.map = self.loadMap()
        self.markArea(start, 1)

    def markArea(self, center, value):
        self.exploredMap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = value

    def markRobot(self):
        # clear robot markings
        self.exploredMap[np.where(self.exploredMap == 5)] = 1
        self.exploredMap[np.where(self.exploredMap == 6)] = 1
        self.markArea(self.center, 5)
        r, c = self.center
        if self.direction == NORTH:
            self.exploredMap[r-1, c] = 6
        elif self.direction == EAST:
            self.exploredMap[r, c+1] = 6
        elif self.direction == SOUTH:
            self.exploredMap[r+1, c] = 6
        else:
            self.exploredMap[r, c-1] = 6

    def loadMap(self):
        with open(os.path.join('Maps', self.realMap)) as f:
            return np.genfromtxt(f, dtype=int, delimiter=1)

    def canMove(self):
        r, c = self.robotCenter
        x, y = np.meshgrid([-1, 0, 1], [-1, 0, 1])
        x, y = x+r, y+c
        if (np.any(x < 0) or np.any(y < 0) or np.any(x >= MAX_ROWS) or np.any(y >= MAX_COLS)):
            return False
        return True

    def getValue(self, inds):
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
        sensors = np.asarray([[None]*4]*6)
        maxDistance = 4
        r, c = self.center

        # FL
        if self.direction == NORTH:
            sensors[0] = self.getValue(zip(range(r-maxDistance-1, r-1), [c-1]*maxDistance))
        elif self.direction == EAST:
            sensors[0] = self.getValue(zip([r-1]*maxDistance, range(c+1, c+maxDistance+1)))
        elif self.direction == WEST:
            sensors[0] = self.getValue(zip([r+1]*maxDistance, range(c-maxDistance-1, c-1)))
        else:
            sensors[0] = self.getValue(zip(range(r+1, r+maxDistance+1), [c+1]*maxDistance))

        # FM
        if self.direction == NORTH:
            sensors[1] = self.getValue(zip(range(r-maxDistance-1, r-1), [c]*maxDistance))
        elif self.direction == EAST:
            sensors[1] = self.getValue(zip([r]*maxDistance, range(c+1, c+maxDistance+1)))
        elif self.direction == WEST:
            sensors[1] = self.getValue(zip([r]*maxDistance, range(c-maxDistance-1, c-1)))
        else:
            sensors[1] = self.getValue(zip(range(r+1, r+maxDistance+1), [c]*maxDistance))

        # FR
        if self.direction == NORTH:
            sensors[2] = self.getValue(zip(range(r-maxDistance-1, r-1), [c+1]*maxDistance))
        elif self.direction == EAST:
            sensors[2] = self.getValue(zip([r+1]*maxDistance, range(c+1, c+maxDistance+1)))
        elif self.direction == WEST:
            sensors[2] = self.getValue(zip([r-1]*maxDistance, range(c-maxDistance-1, c-1)))
        else:
            sensors[2] = self.getValue(zip(range(r+1, r+maxDistance+1), [c-1]*maxDistance))

        # RT
        if self.direction == NORTH:
            sensors[3] = self.getValue(zip([r-1]*maxDistance, range(c+2, c+maxDistance+2)))
        elif self.direction == EAST:
            sensors[3] = self.getValue(zip(range(r+2, r+maxDistance+2), [c+1]*maxDistance))
        elif self.direction == WEST:
            sensors[3] = self.getValue(zip(range(r-maxDistance-2, r-2), [c-1]*maxDistance))
        else:
            sensors[3] = self.getValue(zip([r+1]*maxDistance, range(c-maxDistance-2, c-2)))

        # RB
        if self.direction == NORTH:
            sensors[4] = self.getValue(zip([r+1]*maxDistance, range(c+2, c+maxDistance+2)))
        elif self.direction == EAST:
            sensors[4] = self.getValue(zip(range(r+2, r+maxDistance+2), [c-1]*maxDistance))
        elif self.direction == WEST:
            sensors[4] = self.getValue(zip(range(r-maxDistance-2, r-2), [c+1]*maxDistance))
        else:
            sensors[4] = self.getValue(zip([r-1]*maxDistance, range(c-maxDistance-2, c-2)))

        # LT
        if self.direction == NORTH:
            sensors[5] = self.getValue(zip([r-1]*maxDistance, range(c-maxDistance-2, c-2)))
        elif self.direction == EAST:
            sensors[5] = self.getValue(zip(range(r-maxDistance-2, r-2), [c+1]*maxDistance))
        elif self.direction == WEST:
            sensors[5] = self.getValue(zip(range(r+2, r+maxDistance+2), [c-1]*maxDistance))
        else:
            sensors[5] = self.getValue(zip([r+1]*maxDistance, range(c+2, c+maxDistance+2)))

        return sensors

    def moveBot(self, movement):
        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
                self.head = self.head + [1, 1]
                # self.markRobot()
            elif movement == LEFT:
                self.direction = WEST
                self.head = self.head + [1, -1]
                # self.markRobot()
            else:
                self.head = self.head + [-1, 0]
                self.center = self.center + [-1, 0]
                self.markArea(self.center, 1)
                # self.markRobot()
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
                self.head = self.head + [1, -1]
                # self.markRobot()
            elif movement == LEFT:
                self.direction = NORTH
                self.head = self.head + [-1, -1]
                # self.markRobot()
            else:
                self.head = self.head + [0, 1]
                self.center = self.center + [0, 1]
                self.markArea(self.center, 1)
                # self.markRobot()
        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
                self.head = self.head + [-1, -1]
                # self.markRobot()
            elif movement == LEFT:
                self.direction = EAST
                self.head = self.head + [-1, 1]
                # self.markRobot()
            else:
                self.head = self.head + [1, 0]
                self.center = self.center + [1, 0]
                self.markArea(self.center, 1)
                # self.markRobot()
        else:
            if movement == RIGHT:
                self.direction = NORTH
                self.head = self.head + [-1, 1]
                # self.markRobot()
            elif movement == LEFT:
                self.direction = SOUTH
                self.head = self.head + [1, 1]
                # self.markRobot()
            else:
                self.head = self.head + [0, -1]
                self.center = self.center + [0, -1]
                self.markArea(self.center, 1)
                # self.markRobot()
