import random
import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
import time
from .prm import PRM_planning
from .voronoi import VRM_planning


start = (0, 0)
goal = (0, 0)
robot_size = 1


def getPathLength(path):
    length = 0
    for i in range(1, len(path)):
        diff_x = path[i][0] - path[i-1][0]
        diff_y = path[i][1] - path[i-1][1]
        length += math.sqrt((diff_x**2)+(diff_y**2))
    return length


def getMetrics(map):
    time1 = time.time()
    vrm_path = VRM_planning(
        start[0], start[1], goal[0], goal[1], map[0], map[1], robot_size)
    time2 = time.time()
    prm_path = PRM_planning(
        start[0], start[1], goal[0], goal[1], map[0], map[1], robot_size)
    time3 = time.time()
    return [getPathLength(vrm_path), (time2-time1), getPathLength(prm_path), (time3-time2)]
