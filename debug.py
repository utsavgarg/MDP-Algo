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

data = "COMPUTE|35|35|35|35|60|4"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|5|5|35"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|4|5|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|5|5|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|4|5|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|4|5|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|27|28|26|4|5|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|17|18|17|5|4|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|8|8|8|9|4|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|8|8|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|8|8|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|8|8|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|35|7|41"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|35|7|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|35|44|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|35|35|35|3|60|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)


data = "COMPUTE|27|27|28|35|60|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|18|18|18|35|3|60"
split_data = data.split("|")
sensors = map(float, split_data[1:])
current_pos = exp.robot.center
current = exp.moveStep(sensors)
move = current[0]
currentPos = tuple(exp.robot.center)
markArea(exp.currentMap, exp.robot.center, exp.robot.head)

data = "COMPUTE|9|9|9|35|60|60"
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

data = "COMPUTE|35|35|35|2|18|8"
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
