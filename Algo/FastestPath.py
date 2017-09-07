import copy
import numpy as np

from Constants import MAX_ROWS, MAX_COLS, NORTH, SOUTH, EAST, FORWARD, LEFT, RIGHT
from Queue import PriorityQueue


class FastestPath:
    def __init__(self, exploredMap, start, goal, direction, waypoint=None, sim=True):
        self.exploredMap = exploredMap
        self.start = start
        self.goal = goal
        self.waypoint = waypoint
        self.index = 1
        self.path = []
        if sim:
            from Simulator import Robot
            self.robot = Robot(self.exploredMap, direction, start, None)

    def __getHeuristicCosts(self, goal):
        # calculates Manhattan distance
        cols, rows = np.meshgrid(range(0, 15), range(0, 20))
        cost = np.zeros([20, 15])
        cost = np.abs(rows - goal[0]) + np.abs(cols - goal[1])
        return cost

    def __validMove(self, loc):
        r, c = loc
        x, y = np.meshgrid([-1, 0, 1], [-1, 0, 1])
        x, y = x+r, y+c
        if (np.any(x < 0) or np.any(y < 0) or np.any(x >= MAX_ROWS) or np.any(y >= MAX_COLS)):
            return False
        elif (np.any(self.exploredMap[x[0, 0]:x[0, 2]+1, y[0, 0]:y[2, 0]+1] != 1)):
            return False
        return True

    def __getNeighbours(self, loc, path):
        inds = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbours = []
        for ind in inds:
            if ((0 <= ind[0]+loc[0] < MAX_ROWS) and (0 <= ind[1]+loc[1] < MAX_COLS)):
                if (self.exploredMap[ind[0]+loc[0], ind[1]+loc[1]] == 1):
                    if not ([ind[0]+loc[0], ind[1]+loc[1]] in path):
                        if (self.__validMove([ind[0]+loc[0], ind[1]+loc[1]])):
                            neighbours.append([ind[0]+loc[0], ind[1]+loc[1]])
        return neighbours

    def __greedy(self, start, goal, h_n):
        path = []
        if type(start) is np.ndarray:
            start = start.tolist()
        currentLocation = copy.copy(start)
        path.append(currentLocation)
        if type(goal) is np.ndarray:
            goal = goal.tolist()
        while (currentLocation != goal):
            q = PriorityQueue()
            neighbours = self.__getNeighbours(currentLocation, path)
            for neighbour in neighbours:
                q.put((h_n[tuple(neighbour)], neighbour))
            currentLocation = q.get()[1]
            path.append(currentLocation)
            print path
        return path

    def getFastestPath(self):
        path = []
        start = copy.copy(self.start)
        if (self.waypoint):
            h_n = self.__getHeuristicCosts(self.waypoint)
            fsp = self.__greedy(start, self.waypoint, h_n)
            start = copy.copy(self.waypoint)
            path.extend(fsp)
            self.start = self.waypoint
        h_n = self.__getHeuristicCosts(self.goal)
        fsp = self.__greedy(start, self.goal, h_n)
        path.extend(fsp)
        self.path = path
        self.markMap()

    def markMap(self):
        for ind in self.path:
            self.exploredMap[tuple(ind)] = 6

    def moveStep(self):
        movement = []
        if (self.robot.center.tolist() != self.path[self.index]):
            diff = self.robot.center - np.asarray(self.path[self.index])
            if (diff[0] == -1 and diff[1] == 0):  # Going south
                if self.robot.direction == NORTH:
                    movement.extend((RIGHT, RIGHT, FORWARD))
                elif self.robot.direction == EAST:
                    movement.extend((RIGHT, FORWARD))
                elif self.robot.direction == SOUTH:
                    movement.append(FORWARD)
                else:
                    movement.extend((LEFT, FORWARD))
            elif (diff[0] == 0 and diff[1] == 1):  # Going west
                if self.robot.direction == NORTH:
                    movement.extend((LEFT, FORWARD))
                elif self.robot.direction == EAST:
                    movement.extend((RIGHT, RIGHT, FORWARD))
                elif self.robot.direction == SOUTH:
                    movement.extend((RIGHT, FORWARD))
                else:
                    movement.append(FORWARD)
            elif (diff[0] == 0 and diff[1] == -1):  # Going east
                if self.robot.direction == NORTH:
                    movement.extend((RIGHT, FORWARD))
                elif self.robot.direction == EAST:
                    movement.append(FORWARD)
                elif self.robot.direction == SOUTH:
                    movement.extend((LEFT, FORWARD))
                else:
                    movement.extend((RIGHT, RIGHT, FORWARD))
            else:  # Going north
                if self.robot.direction == NORTH:
                    movement.append(FORWARD)
                elif self.robot.direction == EAST:
                    movement.extend((LEFT, FORWARD))
                elif self.robot.direction == SOUTH:
                    movement.extend((RIGHT, RIGHT, FORWARD))
                else:
                    movement.extend((RIGHT, FORWARD))
            for move in movement:
                self.robot.moveBot(move)
        self.index += 1
