output = open('debug_check.py', 'w')

output.write("import os\n" +
             "import numpy as np\n" +
             "from Algo.Exploration import Exploration\n" +
             "def markArea(curmap, center, head):\n" +
             "\tcurmap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = 5\n" +
             "\tcurmap[head[0], head[1]] = 6\n" +
             "\tprint curmap.astype(int)\n" +
             "\tprint ''\n" +
             "\tcurmap[center[0]-1:center[0]+2, center[1]-1:center[1]+2] = 1\n" +
             "currentMap = np.zeros([20, 15])\n\n" +
             "exp = Exploration(sim=False)\n" +
             "current_pos = exp.robot.center\n" +
             "markArea(exp.currentMap, exp.robot.center, exp.robot.head)\n")

with open('log.txt') as log:
    for line in log:
        if ('COMPUTE' in line):
            output.write("\n" +
                         "data = '"+line.strip()+"'\n" +
                         "print data\n" +
                         "split_data = data.split('|')\n" +
                         "sensors = map(float, split_data[1:])\n" +
                         "current_pos = exp.robot.center\n" +
                         "current = exp.moveStep(sensors)\n" +
                         "move = current[0]\n" +
                         "currentPos = tuple(exp.robot.center)\n" +
                         "markArea(exp.currentMap, exp.robot.center, exp.robot.head)\n")

output.close()
