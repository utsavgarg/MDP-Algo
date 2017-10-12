import os
import numpy as np

from Algo.Exploration import Exploration


def markArea(curmap, center, head):
        curmap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = 5
        curmap[head[0], head[1]] = 6
        print curmap
        curmap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = 1

currentMap = np.zeros([20, 15])

exp = Exploration(sim=False)
current_pos = exp.robot.center
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|24|35|35|4|3|58"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|15|25|35|4|5|16"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|0|14|35|3|3|6"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|24|25|24|35|6|4"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|15|15|14|35|16|3"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|6|5|25|14"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|0|0|0|35|48|13"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|6|26|4"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|26|35|16|4"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|15|35|6|3"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|6|35|60|4"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|4|54|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|14|54|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|26|35|23|3|38"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|17|35|35|14|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|24|14|0|4|43|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)


data = "COMPUTE|7|35|35|35|15|37"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|16|0|3|55|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|25|35|35|35|15|4"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|24|25|5|3|23|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|18|35|35|23|26|14"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

#########################################################

data = "COMPUTE|35|35|35|3|2|8"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|2|60|8"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)
