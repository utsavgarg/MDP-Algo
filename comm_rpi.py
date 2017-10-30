#!/usr/bin/env python
"""Tornado server to run the simulation experiments

Attributes:
    app (tornado.web.Application): Address mappings
    clients (dict): Dictionary of active clients
    settings (dict): Settings for the web-server
"""

import socket
import json
import numpy as np
import os
import time
import tornado.web as web
import tornado.websocket as websocket
import tornado.ioloop as ioloop
import threading
from threading import Thread

from tornado.options import define, options
from Algo.Exploration import Exploration
from Algo.FastestPath import FastestPath
from Algo.Constants import START, GOAL, NORTH, WEST

__author__ = "Utsav Garg"

# Global Variables
define("port", default=8888, help="run on the given port", type=int)
clients = dict()
currentMap = np.zeros([20, 15])
mdfCounter = 3


# def loadMap():
#     with open(os.path.join('Maps', 'sample_wk11.txt')) as f:
#         return np.genfromtxt(f, dtype=int, delimiter=1)

# currentMap = loadMap()

log_file = open('log.txt', 'w')

area = 0
exp = ''
fsp = ''
visited = dict()
waypoint = None
steps = 0
numCycle = 1
t_s = 0
direction = 1

map_name = 'map.txt'

step = 0.1


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
        global step
        step = float(self.step)
        startExploration(self.limit, self.coverage)
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
               START, GOAL, 0)


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


def startExploration(limit, coverage):
    """To start the exploration of the maze
    """
    global exp, t_s
    exp = Exploration(map_name, 5)
    t_s = time.time()
    t2 = FuncThread(exploration, exp, limit, coverage)
    t2.start()
    # t2.join()


def exploration(exp, limit, coverage):
    """To explore the map and update the front-end after each move

    Args:
        exp (Exploration): New instance of the exploration class
    """
    global currentMap, area
    limit = map(int, str(limit).strip().split(':'))
    time_limit = limit[0]*60*60 + limit[1]*60 + limit[2]
    elapsedTime = 0
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL, 0)
    logger('Exploration Started !')
    current = exp.moveStep()
    currentMap = exp.currentMap
    area = exp.exploredArea
    visited = dict()
    steps = 0
    numCycle = 1
    while (not current[1] and elapsedTime <= time_limit and exp.exploredArea < int(coverage)):
        elapsedTime = round(time.time()-t_s, 2)
        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
               elapsedTime)
        current = exp.moveStep()
        currentMap = exp.currentMap
        area = exp.exploredArea
        steps += 1
        currentPos = tuple(exp.robot.center)
        if (currentPos in visited):
            visited[currentPos] += 1
            if (visited[currentPos] > 3):
                neighbour = exp.getExploredNeighbour()
                if (neighbour):
                    neighbour = np.asarray(neighbour)
                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                      exp.robot.direction, None)
                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                    exp.robot.center = neighbour
                else:
                    break
        else:
            visited[currentPos] = 1
        if (np.array_equal(exp.robot.center, START)):
            numCycle += 1
            if (numCycle > 1 and steps > 4):
                neighbour = exp.getExploredNeighbour()
                if (neighbour):
                    neighbour = np.asarray(neighbour)
                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                      exp.robot.direction, None)
                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                    exp.robot.center = neighbour
                else:
                    break
        time.sleep(float(step))
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
           elapsedTime)
    logger('Exploration Done !')
    logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
    logger("Map Descriptor 2  -->  "+str(exp.robot.descriptor_2()))
    print currentMap
    fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction, None)
    logger('Fastest Path Started !')
    fastestPath(fsp, START, exp.exploredArea, None)


def startFastestPath(waypoint):
    """To start the fastest path of the maze
    """
    global fsp
    global t_s
    waypoint = map(int, waypoint)
    fsp = FastestPath(currentMap, START, GOAL, NORTH, waypoint)
    t_s = time.time()
    logger('Fastest Path Started !')
    t3 = FuncThread(fastestPath, fsp, GOAL, area, waypoint)
    t3.start()
    # t3.join() this causes the thread to close after exploration and websocket closes


def markMap(curMap, waypoint):
    if waypoint:
        curMap[tuple(waypoint)] = 7
    return curMap


def combineMovement(movement):
    counter = 0
    shortMove = []
    while (counter < len(movement)):
        if (counter < len(movement)-7) and all(x == 'W' for x in movement[counter:counter+7]):
            shortMove.append('Q')
            counter += 7
        elif (counter < len(movement)-5) and all(x == 'W' for x in movement[counter:counter+5]):
            shortMove.append('K')
            counter += 5
        elif (counter < len(movement)-3) and all(x == 'W' for x in movement[counter:counter+3]):
            shortMove.append('X')
            counter += 3
        elif (counter < len(movement)-2) and all(x == 'W' for x in movement[counter:counter+2]):
            shortMove.append('P')
            counter += 2
        else:
            shortMove.append(movement[counter])
            counter += 1
    shortMove += movement[counter:]
    return shortMove


def fastestPath(fsp, goal, area, waypoint):
    fsp.getFastestPath()
    logger(json.dumps(fsp.path))
    while (fsp.robot.center.tolist() != goal.tolist()):
        fsp.moveStep()
        update(markMap(np.copy(fsp.exploredMap), waypoint), area, fsp.robot.center, fsp.robot.head,
               START, GOAL, 0)
    logger('Fastest Path Done !')


def update(current_map, exploredArea, center, head, start, goal, elapsedTime):
    """To send messages to update the front-end

    Args:
        current_map (Numpy array): Current state of the exploration map
        exploredArea (int): Number of cells that have been explored
        center (list): Location of center of the robot
        head (list): Location of head of the robot
        start (list): Location of the starting point for the robot
        goal (list): Location of the finishing point for the robot
        elapsedTime (float): The time that has elapsed since exploration started
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
        clients[key]['object'].write_message(json.dumps(message))


def logger(message):
    for key in clients:
        log = {'log': message}
        clients[key]['object'].write_message(json.dumps(log))


def output_formatter(msg, movement):
    if not isinstance(movement, list):
        movement = movement.tolist()
    movement = map(str, movement)
    return msg+'|'+'|'.join(movement)


class RPi(threading.Thread):
    def __init__(self):
        print "starting rpi communication"
        threading.Thread.__init__(self)

        self.ip = "192.168.26.1"  # Connecting to IP address of MDPGrp26
        self.port = 5182

        # Create a TCP/IP socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        print "sent connection request"

        # Receive and send data to RPi data
    def receive_send(self):
        time_t = time.time()
        while True:
            current_pos = None
            data = self.client_socket.recv(2048)
            log_file.write(data+'\n')
            log_file.flush()
            if (data):
                print ('Received %s from RPi' % (data))
                split_data = data.split("|")
                global exp, t_s, area, steps, numCycle, currentMap, exp, fsp
                if (split_data[0] == 'EXPLORE'):
                    t_s = time.time()
                    exp = Exploration(sim=False)
                    current_pos = exp.robot.center
                    visited[tuple(current_pos)] = 1
                    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                           START, GOAL, 0)
                elif (split_data[0] == 'WAYPOINT'):
                    global waypoint
                    waypoint = map(int, split_data[1:])
                    waypoint[0] = 19 - waypoint[0]
                elif (split_data[0] == 'COMPUTE'):
                    print 'Time 0: %s s' % (time.time() - time_t)
                    sensors = map(float, split_data[1:])
                    current_pos = exp.robot.center
                    current = exp.moveStep(sensors)
                    currentMap = exp.currentMap
                    if (not current[1]):
                        time_t = time.time()
                        move = combineMovement(current[0])
                        elapsedTime = round(time.time()-t_s, 2)
                        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                               START, GOAL, elapsedTime)
                        steps += 1
                        current_pos = tuple(exp.robot.center)
                        if (current_pos in visited):
                            visited[current_pos] += 1
                            if (visited[current_pos] > 3):
                                neighbour = exp.getExploredNeighbour()
                                if (neighbour):
                                    neighbour = np.asarray(neighbour)
                                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                      exp.robot.direction, None, sim=False)
                                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                                    move.extend(combineMovement(fsp.movement))
                                    exp.robot.phase = 2
                                    exp.robot.center = neighbour
                                    exp.robot.head = fsp.robot.head
                                    exp.robot.direction = fsp.robot.direction
                                    currentMap = exp.currentMap
                            if (np.array_equal(exp.robot.center, START) and exp.exploredArea > 50):
                                numCycle += 1
                                if (numCycle > 1 and steps > 4):
                                    neighbour = exp.getExploredNeighbour()
                                    if (neighbour):
                                        neighbour = np.asarray(neighbour)
                                        fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                          exp.robot.direction, None, sim=False)
                                        fastestPath(fsp, neighbour, exp.exploredArea, None)
                                        move.extend(combineMovement(fsp.movement))
                                        exp.robot.phase = 2
                                        exp.robot.center = neighbour
                                        exp.robot.head = fsp.robot.head
                                        exp.robot.direction = fsp.robot.direction
                                        currentMap = exp.currentMap
                        print 'Time 1: %s s' % (time.time() - time_t)
                        time_t = time.time()
                        global mdfCounter
                        if mdfCounter == 3:
                            get_msg = output_formatter('MOVEMENT', [str(exp.robot.descriptor_1()),
                                                       str(exp.robot.descriptor_2())] + move + ['S'])
                            mdfCounter = 0
                        else:
                            get_msg = output_formatter('MOVEMENT', [str(0), str(0)] + move + ['S'])
                            mdfCounter += 1
                        print 'Time 2: %s s' % (time.time() - time_t)
                    else:
                        move = combineMovement(current[0])
                        get_msg = output_formatter('MOVEMENT', [str(exp.robot.descriptor_1()),
                                                   str(exp.robot.descriptor_2())] + move)
                        self.client_socket.send(get_msg)
                        print ('Sent %s to RPi' % (get_msg))
                        log_file.write('Robot Center: %s\n' % (str(exp.robot.center)))
                        log_file.write('Sent %s to RPi\n\n' % (get_msg))
                        log_file.flush()
                        time.sleep(2)
                        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                               START, GOAL, elapsedTime)
                        logger('Exploration Done !')
                        logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
                        logger("Map Descriptor 2  -->  "+str(exp.robot.descriptor_2()))
                        fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction,
                                          None, sim=False)
                        logger('Fastest Path Started !')
                        fastestPath(fsp, START, exp.exploredArea, None)
                        move = combineMovement(fsp.movement)
                        currentMap = exp.currentMap
                        global direction
                        if (fsp.robot.direction == WEST):
                            calibrate_move = ['A', 'L', 'O']
                        else:
                            calibrate_move = ['L', 'O']
                        direction = NORTH
                        get_msg = output_formatter('DONE', [str(exp.robot.descriptor_1()),
                                                   str(exp.robot.descriptor_2())] + ['N'] + move +
                                                   calibrate_move)
                        self.client_socket.send(get_msg)
                        time.sleep(1)
                        get_msg = output_formatter('MOVEMENT', [str(exp.robot.descriptor_1()),
                                                   str(exp.robot.descriptor_2())] + ['N'] + move +
                                                   calibrate_move)
                    self.client_socket.send(get_msg)
                    print ('Sent %s to RPi' % (get_msg))
                    time_t = time.time()
                    log_file.write('Robot Center: %s\n' % (str(exp.robot.center)))
                    log_file.write('Sent %s to RPi\n\n' % (get_msg))
                    log_file.flush()
                elif (split_data[0] == 'FASTEST'):
                    print currentMap
                    fsp = FastestPath(currentMap, START, GOAL, direction, waypoint, sim=False)
                    current_pos = fsp.robot.center
                    fastestPath(fsp, GOAL, 300, waypoint)
                    # move = fsp.movement
                    move = combineMovement(fsp.movement)
                    get_msg = output_formatter('FASTEST', ['N'] + move + ['C'])
                    self.client_socket.send(get_msg)
                    print ('Sent %s to RPi' % (get_msg))
                elif (split_data[0] == 'MANUAL'):
                    manual_movement = split_data[1:]
                    for move in manual_movement:
                        exp.robot.moveBot(move)

    def keep_main(self):
        while True:
            time.sleep(0.5)


settings = dict(
    template_path=os.path.join(os.path.dirname(__file__), "GUI", "templates"),
    debug=True
)

app = web.Application([
    (r'/', IndexHandler),
    (r'/websocket', WebSocketHandler),
    (r'/start', StartHandler),
    (r'/reset', ResetHandler),
    (r'/fsp', FSPHandler),
    (r'/lm', LoadMapHandler),
    (r'/(.*)', web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})
], **settings)


def func1():
    print "Starting communication with RPi"
    client_rpi = RPi()
    rt = threading.Thread(target=client_rpi.receive_send)
    rt.daemon = True
    rt.start()
    client_rpi.keep_main()


def func2():
    print "Starting communication with front-end"
    app.listen(options.port)
    t1 = FuncThread(ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()

if __name__ == '__main__':
    Thread(target=func1).start()
    Thread(target=func2).start()
