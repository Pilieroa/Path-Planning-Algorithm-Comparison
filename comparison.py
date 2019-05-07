import math
import matplotlib.pyplot as plt
import time
from prm import *
from voronoi import *


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


def getPRMValues(map, samples):
    length = 0
    time = 0
    failures = 0
    for i in range(100):
        t1 = time.time()
        path = prm(map[0], map[1], samples)
        t2 = time.time()
        if(path == 'Cannot found path'):
            failures += 1
            continue
        length += getPathLength(path)
        time += t2-t1
    return length/(100 - failures), time/(100 - failures), failures

def getMetrics(map):
    time1 = time.time()
    vrm_path = voronoi(map[0],map[1])
    time2 = time.time()
    prm_len_500, prm_time_500, failures_500  = getPRMValues(map, 500)
    prm_len_800, prm_time_800, failures_800  = getPRMValues(map, 800)
    prm_len_1000, prm_time_1000, failures_1000  = getPRMValues(map, 1000)
    return [getPathLength(vrm_path), (time2-time1), prm_len_500, prm_time_500, failures_500, prm_len_800, prm_time_800, failures_800, prm_len_1000, prm_time_1000, failures_1000]


def makePlots(data, name):
    labels = ['500 Samples', '800 Samples', '1000 Samples']

    lengths = [data[2], data[5], data[8]]
    times = [data[3], data[6], data[9]]
    failures = [data[4], data[7], data[10]]

    plot1 = plt.bar([0,1,2], lengths, 0.25, alpha=0.5, color='b', label='Path Length')
    plot2 = plt.bar([0.25, 1.25,2.25], times, 0.25, alpha=0.5, color='g', label='Runtime')
    plot3 = plt.bar([0.5, 1.5, 2.5], failures, 0.25, alpha=0.5, color='r', label='Percent Failure')
    plot4 = plt.hlines(data[0], xmin=0, xmax=3, color='b', linestyles={'dashed'}, label='Voronoi Path Length')
    plot5 =plt.hlines(data[1], xmin=0, xmax=3, color='g', linestyles={'dashed'}, label='Voronoi Runtime')

    plt.title(name)
    plt.xticks([0.375, 1.375, 2.375], labels)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    makePlots([10,20, 15, 15, 5, 25, 25, 10, 50, 50, 15], 'Sample map')
