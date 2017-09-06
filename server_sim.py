#!/usr/bin/env python
"""Tornado server to run the simulation experiments

Attributes:
    app (tornado.web.Application): Address mappings
    clients (dict): Dictionary of active clients
    settings (dict): Settings for the web-server
"""
import json
import os
import time
import tornado.web as web
import tornado.websocket as websocket
import tornado.ioloop as ioloop
import threading

from tornado.options import define, options
from Algo.Exploration import Exploration
from Algo.Constants import START, GOAL

__author__ = "Utsav Garg"

define("port", default=8888, help="run on the given port", type=int)

clients = dict()


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

    def on_message(self, message):
        """Displays any message received

        Args:
            message (string): Message received from front-end
        """
        print ("Client " + str(self.id) + " received a message : " + str(message))

    def on_close(self):
        """Run when the web socket is closed
        """
        print("WebSocket closed")


class StartHandler(web.RequestHandler):

    """Handles the start of exploration for the maze
    """

    @web.asynchronous
    def get(self):
        self.write("Starting...")
        startExploration()
        self.flush()


class ResetHandler(web.RequestHandler):

    """Handles the reset of the current map
    """

    @web.asynchronous
    def get(self):
        self.write("Reset...")
        global exp
        exp = Exploration('map.txt', 5)
        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
               START, GOAL, 0, '')


class StopHandler(web.RequestHandler):

    """To Stop the robot exploration in the middle
    """

    @web.asynchronous
    def get(self):
        global started
        started = False
        self.flush()


def startExploration():
    """To start the exploration of the maze
    """
    global exp
    global t_s
    exp = Exploration('map.txt', 5)
    t_s = time.time()
    print 'Exploration Started !'
    t2 = FuncThread(exploration, exp)
    t2.start()
    t2.join()


def exploration(exp):
    """To explore the map and update the front-end after each move

    Args:
        exp (Exploration): New instance of the exploration class
    """
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL, 0, '')
    current = exp.moveStep()
    steps = 0
    while (not current and steps < 150):
        elapsedTime = round(time.time()-t_s, 2)
        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
               elapsedTime, exp.robot.movement)
        current = exp.moveStep()
        steps += 1
        time.sleep(0.1)
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
           elapsedTime, exp.robot.movement)
    print 'Exploration Done !'


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
        message['area'] = exploredArea
        tempMap = current_map.copy()
        tempMap[start[0]-1: start[0]+2, start[1]-1: start[1]+2] = 3
        tempMap[goal[0]-1: goal[0]+2, goal[1]-1: goal[1]+2] = 4
        message['map'] = json.dumps(str(tempMap))
        message['center'] = str(center.tolist())[1:-1]
        message['head'] = str(head.tolist())[1:-1]
        message['time'] = elapsedTime
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
    (r'/(.*)', web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})
], **settings)

if __name__ == '__main__':
    app.listen(options.port)
    t1 = FuncThread(ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()
