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

define("port", default=8888, help="run on the given port", type=int)

clients = dict()


class FuncThread(threading.Thread):
    def __init__(self, target, *args):
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        self._target(*self._args)


class IndexHandler(web.RequestHandler):
    @web.asynchronous
    def get(self):
        self.render("index.html")


class WebSocketHandler(websocket.WebSocketHandler):
    def open(self):
        self.id = self.get_argument("Id")
        self.stream.set_nodelay(True)
        clients[self.id] = {"id": self.id, "object": self}

    def on_message(self, message):
        print ("Client " + str(self.id) + " received a message : " + str(message))

    def on_close(self):
        print("WebSocket closed")


class StartHandler(web.RequestHandler):
    @web.asynchronous
    def get(self):
        self.write("Starting...")
        startExploration()
        self.flush()


class ResetHandler(web.RequestHandler):
    @web.asynchronous
    def get(self):
        self.write("Reset...")
        global exp
        exp = Exploration('map.txt', 5)
        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL)


class StopHandler(web.RequestHandler):
    @web.asynchronous
    def get(self):
        global started
        started = False
        self.flush()


def startExploration():
    global exp
    global t_s
    exp = Exploration('map.txt', 5)
    t_s = time.time()
    print 'Exploration Started !'
    t2 = FuncThread(exploration, exp)
    t2.start()
    t2.join()


def exploration(exp):
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
