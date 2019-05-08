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
    for i in range(1, len(path[0])):
        diff_x = path[0][i] - path[0][i-1]
        diff_y = path[1][i] - path[1][i-1]
        length += math.sqrt((diff_x**2)+(diff_y**2))
    return length


def getPRMValues(map, samples):
    length = 0
    net_time = 0
    failures = 0
    for i in range(100):
        print("PRM" + str(i))
        t1 = time.time()
        path = prm(map[0], map[1], samples)
        t2 = time.time()
        if(len(path[0]) == 1):
            failures += 1
            print('Failed!')
            continue
        length += getPathLength(path)
        net_time += t2-t1
    return length/(100 - failures), net_time/(100 - failures), failures


def getMetrics(map):
    time1 = time.time()
    vrm_path = voronoi(map[0], map[1])
    time2 = time.time()
    prm_len_500, prm_time_500, failures_500 = getPRMValues(map, 500)
    prm_len_800, prm_time_800, failures_800 = getPRMValues(map, 800)
    prm_len_1000, prm_time_1000, failures_1000 = getPRMValues(map, 1000)
    return [getPathLength(vrm_path), (time2-time1), prm_len_500, prm_time_500, failures_500, prm_len_800, prm_time_800, failures_800, prm_len_1000, prm_time_1000, failures_1000]

def autolabel(rects, axes, xpos='center'):
    """
    Attach a text label above each bar in *rects*, displaying its height.

    *xpos* indicates which side to place the text w.r.t. the center of
    the bar. It can be one of the following {'center', 'right', 'left'}.
    """

    xpos = xpos.lower()  # normalize the case of the parameter
    ha = {'center': 'center', 'right': 'left', 'left': 'right'}
    offset = {'center': 0.5, 'right': 0.57, 'left': 0.43}  # x_txt = x + w*off

    for rect in rects:
        height = round(rect.get_height(), 2)
        axes.text(rect.get_x() + rect.get_width()*offset[xpos], 1.01*height,
                '{}'.format(height), ha=ha[xpos], va='bottom')

def makePlots(data, name):
    labels = ['500 Samples', '800 Samples', '1000 Samples']

    lengths = [data[2], data[5], data[8]]
    times = [data[3], data[6], data[9]]
    failures = [data[4], data[7], data[10]]

    fig,ax1 = plt.subplots()

    ax2 = ax1.twinx()

    plot1 = ax1.bar([0, 1, 2], lengths, 0.25, alpha=0.5,
                    color='b', label='Path Length')
    plot2 = ax2.bar([0.25, 1.25, 2.25], times, 0.25,
                    alpha=0.5, color='g', label='Runtime')
    plot3 = ax1.bar([0.5, 1.5, 2.5], failures, 0.25, alpha=0.5,
                    color='r', label='Percent Failure')
    plot4 = ax1.hlines(data[0], xmin=0, xmax=3, color='b', linestyles={
                       'dashed'}, label='Voronoi Path Length')
    plot5 = ax2.hlines(data[1], xmin=0, xmax=3, color='g', linestyles={
                       'dashed'}, label='Voronoi Runtime')

    autolabel(plot1, ax1)
    autolabel(plot2, ax2)
    autolabel(plot3, ax1)
    plt.title(name)
    ax1.set_ylabel('Path length')
    ax2.set_ylabel('Runtime')
    plt.xticks([0.375, 1.375, 2.375], labels)
    fig.legend()
    plt.show()


if __name__ == '__main__':
    ox = []
    oy = []
    obstacle_list_x = []
    obstacle_list_y = []
    for i in range(50):
        ox.append(i)
        oy.append(0.0)
    for i in range(30):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    obstacle_list_x.append(ox)
    obstacle_list_y.append(oy)

    ox = []
    oy = []
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(30, 30, [60, 60], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(30, 40, [30, 25], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(30, 10, [10, 10], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(50, 20, [10, 10], "rectangle")
    obstacle_list_x.append(ox)
    obstacle_list_y.append(oy)

    ox = []
    oy = []
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(30, 60):
        ox.append(10)
        oy.append(i)
    for i in range(5, 60):
        ox.append(30)
        oy.append(i)
    for i in range(0, 2):
        ox.append(40)
        oy.append(i)
    for i in range(0, 40):
        ox.append(45)
        oy.append(i)
    for i in range(10, 60):
        ox.append(48)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(50, 50, [4, 7], "topOpenRectangle")
    obstacle_list_x.append(ox)
    obstacle_list_y.append(oy)

    ox = []
    oy = []
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(30, 30, [60, 60], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(20, 20, [15, 25], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(45, 50, [23, 13], "topOpenRectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(50, 52, [7, 7], "openRectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(20, 53, [10, 10], "openRectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(50, 10, [15, 17], "openRectangle")
    obstacle_list_x.append(ox)
    obstacle_list_y.append(oy)
    names = ["Wide Corridoors", "Easy Obstacles", "Narrow Corridoors", "Difficult Obstacles"]
    for i in range(len(obstacle_list_x)):
        data = getMetrics([obstacle_list_x[i], obstacle_list_y[i]])
        makePlots(data, names[i])
