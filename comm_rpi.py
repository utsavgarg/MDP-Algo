import socket
import time
import json
import numpy as np
import os
import tornado.web as web
import tornado.websocket as websocket
import tornado.ioloop as ioloop
import threading

from tornado.options import define, options
from Algo.Exploration import Exploration
from Algo.FastestPath import FastestPath
from Algo.Constants import START, GOAL

# Global Variables
define("port", default=8890, help="run on the given port", type=int)
clients = dict()
currentMap = np.ones([20, 15])
area = 0
exp = ''
fsp = ''
visited = dict()
steps = 0
numCycle = 1
t_s = 0


def markMap(curMap, waypoint):
    if waypoint:
        curMap[tuple(waypoint)] = 7
    return curMap


def fastestPath(fsp, goal, area, waypoint):
    fsp.getFastestPath()
    logger(json.dumps(fsp.path))
    while (fsp.robot.center.tolist() != goal.tolist()):
        fsp.moveStep()
        elapsedTime = round(time.time()-t_s, 2)
        update(markMap(np.copy(fsp.exploredMap), waypoint), area, fsp.robot.center, fsp.robot.head,
               START, GOAL, elapsedTime)
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


def output_formatter(msg, start, movement):
    if not isinstance(start, list):
        start = start.tolist()
    if not isinstance(movement, list):
        movement = movement.tolist()
    start = map(str, start)
    movement = map(str, movement)
    return msg+'|'+'|'.join(start)+'|'+'|'.join(movement)


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
        while True:
            current_pos = None
            data = self.client_socket.recv(1024)
            if (data):
                print ('Received %s from RPi' % (data))
                split_data = data.split("|")
                global exp, t_s, area, steps, numCycle, currentMap, exp, fsp
                if (split_data[0] == 'EXPLORE'):
                    t_s = time.time()
                    exp = Exploration(sim=False)
                    current_pos = exp.robot.center
                    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                           START, GOAL, 0)
                elif (split_data[0] == 'COMPUTE'):
                    sensors = map(float, split_data[1:])
                    current_pos = exp.robot.center
                    current = exp.moveStep(sensors)
                    if (not current[1]):
                        move = current[0]
                        currentMap = exp.currentMap
                        elapsedTime = round(time.time()-t_s, 2)
                        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                               START, GOAL, elapsedTime)
                        steps += 1
                        currentPos = tuple(exp.robot.center)
                        if (currentPos in visited):
                            visited[currentPos] += 1
                            if (visited[currentPos] > 2):
                                neighbour = exp.getExploredNeighbour()
                                if (neighbour):
                                    neighbour = np.asarray(neighbour)
                                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                      exp.robot.direction, None, sim=False)
                                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                                    move.extend(fsp.movement)
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
                                        fastestPath(fsp, neighbour, exp.exploredArea, None, sim=False)
                                        move.extend(fsp.movement)
                                        exp.robot.center = neighbour
                                    else:
                                        break
                    else:
                        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                               START, GOAL, elapsedTime)
                        logger('Exploration Done !')
                        logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
                        logger("Map Descriptor 2  -->  "+str(exp.robot.descriptor_2()))
                        fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction,
                                          None, sim=False)
                        logger('Fastest Path Started !')
                        fastestPath(fsp, START, exp.exploredArea, None)
                        move.extend(fsp.movement)
                    get_msg = output_formatter('MOVEMENT', current_pos, move)
                    self.client_socket.send(get_msg)
                    print ('Sent %s to RPi' % (get_msg))
                elif (split_data[0] == 'FASTEST'):
                    waypoint = map(int, split_data[1:])
                    fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction,
                                      waypoint, sim=False)
                    current_pos = fsp.robot.center
                    fastestPath(fsp, START, exp.exploredArea, None)
                    move = fsp.movement
                    get_msg = output_formatter('MOVE', current_pos, move)
                    self.client_socket.send(get_msg)
                    print ('Sent %s to RPi' % (get_msg))
                elif (split_data[0] == 'MANUAL'):
                    # need to receive:
                    # starting coordinates
                    # starting direction
                    # movement
                    pass

    def keep_main(self):
        while True:
            time.sleep(0.5)


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


settings = dict(
    template_path=os.path.join(os.path.dirname(__file__), "GUI", "templates"),
    debug=True
)

app = web.Application([
    (r'/', IndexHandler),
    (r'/websocket', WebSocketHandler),
    (r'/(.*)', web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})
], **settings)

if __name__ == "__main__":
    # for rpi
    print "starting rpi comm"
    client_rpi = RPi()
    rt = threading.Thread(target=client_rpi.receive_send)
    rt.daemon = True
    rt.start()
    client_rpi.keep_main()

    # for front end
    app.listen(options.port)
    t1 = FuncThread(ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()
