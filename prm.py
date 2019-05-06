"""
Probablistic Road Map (PRM) Planner
"""

import random
import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt

# parameter
N_SAMPLE = 500  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = True


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
        Search NN
        inp: input data, single frame or multi frame
        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = round(d / D)

    for i in range(nstep):
        idxs, dist = okdtree.search(np.array([x, y]).reshape(2, 1))
        if dist[0] <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.array([gx, gy]).reshape(2, 1))
    if dist[0] <= rr:
        return True  # collision

    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    """
    Road map generation
    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """

    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index, dists = skdtree.search(
            np.array([ix, iy]).reshape(2, 1), k=nsample)
        inds = index[0]
        edge_id = []
        #  print(index)

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    while True:
        if not openset:
            print("Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]

        # show graph
        if show_animation and len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() - minx) * (maxx - minx)
        ty = (random.random() - miny) * (maxy - miny)

        index, dist = obkdtree.search(np.array([tx, ty]).reshape(2, 1))

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


class Obstacle:
    ox = []
    oy = []
    x = 0
    y = 0
    data = []
    shape = ""

    def __init__(self, ox, oy):
        self.ox = ox
        self.oy = oy

    def makeShape(self, x, y, data, shape):
        self.x = x
        self.y = y
        self.data = data
        self.shape = shape

        if self.shape == "rectangle":
            ox, oy = self.makeRectangle()
        if self.shape == "openRectangle":
            ox, oy = self.makeOpenRectangle()
        return ox, oy

    def makeRectangle(self):
        for i in range(self.x-self.data[0]//2, (self.x+self.data[0]//2)+1):
            self.ox.append(i)
            self.oy.append(self.y - self.data[1]//2)
        for i in range(self.x-self.data[0]//2, (self.x + self.data[0]//2)+1):
            self.ox.append(i)
            self.oy.append(self.y + self.data[1]//2)
        for i in range(self.y - self.data[1]//2, (self.y+self.data[1]//2) + 1):
            self.ox.append(self.x - self.data[0]//2)
            self.oy.append(i)
        for i in range(self.y - self.data[1]//2, (self.y+self.data[1]//2) + 1):
            self.ox.append(self.x + self.data[0]//2)
            self.oy.append(i)
        return self.ox, self.oy

    def makeOpenRectangle(self):
        for i in range(self.x-self.data[0]//2, (self.x+self.data[0]//2)+1):
            if i not in range(self.x-1, self.x+2):
                self.ox.append(i)
                self.oy.append(self.y - self.data[1]//2)
        for i in range(self.x-self.data[0]//2, (self.x + self.data[0]//2)+1):
            self.ox.append(i)
            self.oy.append(self.y + self.data[1]//2)
        for i in range(self.y - self.data[1]//2, (self.y+self.data[1]//2) + 1):
            self.ox.append(self.x - self.data[0]//2)
            self.oy.append(i)
        for i in range(self.y - self.data[1]//2, (self.y+self.data[1]//2) + 1):
            self.ox.append(self.x + self.data[0]//2)
            self.oy.append(i)
        return self.ox, self.oy


def prm(ox, oy):
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    robot_size = 1.0  # [m]
    if show_animation:
        plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "^r")
    plt.plot(gx, gy, "^c")
    plt.grid(True)
    plt.axis("equal")

    rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


def main():
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
    for i in range(30):
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
    obstacle_list_x.append(ox)
    obstacle_list_y.append(oy)
    ox = []
    oy = []
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(30, 30, [70, 70], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(30, 30, [20, 20], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(45, 50, [20, 15], "openRectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(50, 50, [10, 10], "openRectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(20, 53, [10, 10], "rectangle")
    obs = Obstacle(ox, oy)
    ox, oy = obs.makeShape(50, 10, [15, 17], "rectangle")
    obstacle_list_x.append(ox)
    obstacle_list_y.append(oy)
    for i in range(1, len(obstacle_list_x)):
        prm(obstacle_list_x[-i], obstacle_list_y[-i])


if __name__ == '__main__':
    main()
