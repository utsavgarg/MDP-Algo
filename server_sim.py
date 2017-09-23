#!/usr/bin/env python
"""Tornado server to run the simulation experiments

Attributes:
    app (tornado.web.Application): Address mappings
    clients (dict): Dictionary of active clients
    settings (dict): Settings for the web-server
"""
import json
import numpy as np
import os
import time
import tornado.web as web
import tornado.websocket as websocket
import tornado.ioloop as ioloop
import threading

from tornado.options import define, options
from Algo.Exploration import Exploration
from Algo.FastestPath import FastestPath
from Algo.Constants import START, GOAL, NORTH

__author__ = "Utsav Garg"

define("port", default=8888, help="run on the given port", type=int)

clients = dict()

currentMap = np.ones([20, 15])
map_name = 'map.txt'

area = 0


class FuncThread(threading.Thread):

    """Class to create and run functions on different threads
    """

    def __init__(self, target, *args):
        """Construction to initialize the thread

        Args:
            target (function): Function to be run on new threads
            *args: arguments to be passed to the function
        """
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        """Overrides run function to run the function with given arguments
        """
        self._target(*self._args)


class IndexHandler(web.RequestHandler):

    """To display the front-end interface
    """

    @web.asynchronous
    def get(self):
        self.render("index.html")


class WebSocketHandler(websocket.WebSocketHandler):

    """Handles web-socket requests from the front-end to receive/send messages

    Attributes:
        id (string): id string from GET request
    """

    def open(self):
        """Open a web socket for communication
        """
        self.id = self.get_argument("Id")
        self.stream.set_nodelay(True)
        clients[self.id] = {"id": self.id, "object": self}
        print("WebSocket opened")

    def on_message(self, message):
        """Displays any message received

        Args:
            message (string): Message received from front-end
        """
        print("Client " + str(self.id) + " received a message : " + str(message))

    def on_close(self):
        """Run when the web socket is closed
        """
        print("WebSocket closed")
        if self.id in clients:
            del clients[self.id]


class StartHandler(web.RequestHandler):

    """Handles the start of exploration for the maze
    """

    @web.asynchronous
    def get(self):
        self.write("Starting...")
        self.step = self.get_argument("step")
        self.limit = self.get_argument("limit")
        self.coverage = self.get_argument("coverage")
        startExploration(self.step, self.limit, self.coverage)
        self.flush()


class ResetHandler(web.RequestHandler):

    """Handles the reset of the current map
    """

    @web.asynchronous
    def get(self):
        self.write("Reset...")
        global exp
        exp = Exploration(map_name, 5)
        update(np.zeros([20, 15]), exp.exploredArea, exp.robot.center, exp.robot.head,
               START, GOAL, 0, '')


class StopHandler(web.RequestHandler):

    """To Stop the robot exploration in the middle
    """

    @web.asynchronous
    def get(self):
        global started
        started = False
        self.flush()


class FSPHandler(web.RequestHandler):

    """Handles the start of fastest path for the maze
    """

    @web.asynchronous
    def get(self):
        self.x = self.get_argument("x")
        self.y = self.get_argument("y")
        self.write("Starting...")
        startFastestPath([self.x, self.y])
        self.flush()


class LoadMapHandler(web.RequestHandler):

    """Handles the start of fastest path for the maze
    """

    @web.asynchronous
    def get(self):
        global map_name
        self.name = self.get_argument("name")
        map_name = self.name


def startExploration(step, limit, coverage):
    """To start the exploration of the maze
    """
    global exp
    global t_s
    exp = Exploration(map_name, 5)
    t_s = time.time()
    print 'Exploration Started !'
    t2 = FuncThread(exploration, exp, step, limit, coverage)
    t2.start()
    # t2.join()


def exploration(exp, step, limit, coverage):
    """To explore the map and update the front-end after each move

    Args:
        exp (Exploration): New instance of the exploration class
    """
    global currentMap
    global area
    limit = map(int, str(limit).strip().split(':'))
    time_limit = limit[0]*60*60 + limit[1]*60
    print time_limit, int(coverage), float(step)
    elapsedTime = 0
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL, 0, '')
    current = exp.moveStep()
    currentMap = exp.currentMap
    area = exp.exploredArea
    steps = 0
    while (not current and elapsedTime <= time_limit and exp.exploredArea < int(coverage)):
        elapsedTime = round(time.time()-t_s, 2)
        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
               elapsedTime, exp.robot.movement)
        current = exp.moveStep()
        currentMap = exp.currentMap
        area = exp.exploredArea
        steps += 1
        time.sleep(float(step))
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
           elapsedTime, exp.robot.movement)
    print 'Exploration Done !'
    print exp.robot.descriptor_1()
    print exp.robot.descriptor_2()
    fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction, None)
    print 'Fastest Path Started !'
    fastestPath(fsp, START, exp.exploredArea, None)


def startFastestPath(waypoint):
    """To start the fastest path of the maze
    """
    global fsp
    global t_s
    waypoint = map(int, waypoint)
    fsp = FastestPath(currentMap, START, GOAL, NORTH, waypoint)
    t_s = time.time()
    print 'Fastest Path Started !'
    t3 = FuncThread(fastestPath, fsp, GOAL, area, waypoint)
    t3.start()
    # t3.join() this causes the thread to close after exploration and websocket closes


def markMap(curMap, waypoint):
    if waypoint:
        curMap[tuple(waypoint)] = 7
    return curMap


def fastestPath(fsp, goal, area, waypoint):
    fsp.getFastestPath()
    print fsp.path
    while (fsp.robot.center.tolist() != goal.tolist()):
        fsp.moveStep()
        elapsedTime = round(time.time()-t_s, 2)
        update(markMap(np.copy(fsp.exploredMap), waypoint), area, fsp.robot.center, fsp.robot.head,
               START, GOAL, elapsedTime, fsp.robot.movement)
        time.sleep(0.1)
    print 'Fastest Path Done !'


def update(current_map, exploredArea, center, head, start, goal, elapsedTime, log):
    """To send messages to update the front-end

    Args:
        current_map (Numpy array): Current state of the exploration map
        exploredArea (int): Number of cells that have been explored
        center (list): Location of center of the robot
        head (list): Location of head of the robot
        start (list): Location of the starting point for the robot
        goal (list): Location of the finishing point for the robot
        elapsedTime (float): The time that has elapsed since exploration started
        log (string): Log of robot movements
    """
    for key in clients:
        message = dict()
        message['area'] = '%.2f' % (exploredArea)
        tempMap = current_map.copy()
        tempMap[start[0]-1: start[0]+2, start[1]-1: start[1]+2] = 3
        tempMap[goal[0]-1: goal[0]+2, goal[1]-1: goal[1]+2] = 4
        message['map'] = json.dumps(tempMap.astype(int).tolist())
        message['center'] = json.dumps(center.astype(int).tolist())
        message['head'] = json.dumps(head.astype(int).tolist())
        message['time'] = '%.2f' % (elapsedTime)
        message['msg'] = str(log)[1:-1]
        clients[key]['object'].write_message(json.dumps(message))


settings = dict(
    template_path=os.path.join(os.path.dirname(__file__), "GUI", "templates"),
    debug=True
)

app = web.Application([
    (r'/', IndexHandler),
    (r'/websocket', WebSocketHandler),
    (r'/start', StartHandler),
    (r'/reset', ResetHandler),
    (r'/stop', StopHandler),
    (r'/fsp', FSPHandler),
    (r'/lm', LoadMapHandler),
    (r'/(.*)', web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})
], **settings)

if __name__ == '__main__':
    app.listen(options.port)
    t1 = FuncThread(ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()
