# Program to load obstacle course for Lab 4 - RRT

# usage:  python rrt.py obstacles_file start_goal_file


from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math
import rrtUtil as RU
import time

class config:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

def rand(dim_x, dim_y):
    # 600 by 600 map
    # Obstacle detection not implemented yet
    random.seed()
    #while True:
    q_rand = config(random.randint(100,dim_x), random.randint(100,dim_y))
    return q_rand

def near(q_rand, G):
    def dist(q1, q2):
        return math.sqrt((q2.y - q1.y)**2 + (q2.x - q1.x)**2)

    min_dist = np.inf
    q = config(0,0)
    for point in G:
        vertex = config(point[0], point[1])
        if dist(vertex, q_rand) < min_dist:
            min_dist = dist(vertex, q_rand)
            q = vertex

    return q

def new(q_near, q_rand, dq):
    def angle(q1, q2):
        return math.atan2(q_rand.y-q_near.y, q_rand.x-q_near.x)

    theta = angle(q_near, q_rand)
    new_x = dq * np.cos(theta) + q_near.x
    new_y = dq * np.sin(theta) + q_near.y

    return config(new_x, new_y)

def tuplefy(q):
    return (q.x, q.y)

def printc(q):
    print(q.x, q.y)

def build_obs_list(obstacle_path):
    '''
        returns a list of obstacles (represented by a list of configs)
    '''
    obs = list()
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        for i in range(quantity):
            ob = list()
            n = int(f.readline())
            for j in range(n):
                line = f.readline().strip().split(' ')
                ob.append(config(int(line[0]), int(line[1])))
            obs.append(ob)
    return obs


if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    args = parser.parse_args()


    q_start = config(56.0,18.0) #otherwise, start
    G = {tuplefy(q_start):[]}
    q_goal = config(448.0, 542.0)
    goal_found = False

    obs = build_obs_list(args.obstacle_path)

    while True:
        if goal_found:
            break
        k = len(G)
        for i in range(k):
            while True:
                q_rand = rand(600, 600)
                inobs = False
                for ob in obs:
                    if RU.is_inside(ob, len(ob), q_rand):
                        inobs = True
                        break

                if not inobs:
                    break

            q_near = near(q_rand, G)

            q_new = new(q_near, q_rand, 5)
            inobs2 = False
            for ob in obs:
                if RU.is_inside(ob, len(ob), q_new):
                    inobs2 = True
                    break
            if inobs2:
                continue
            printc(q_rand)
            printc(q_near)
            printc(q_new)

            print("\n")


            if abs(q_goal.x - q_new.x) < 5 and abs(q_goal.y - q_new.y) < 5:
                printc(q_new)
                print("FOUND GOAL")
                goal_found = True
                break

            G[tuplefy(q_new)] = []
            G[tuplefy(q_near)].append(tuplefy(q_new))

        #print(G)
        #time.sleep(2)


        #print(G)

    #print(G)

    '''
    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)

    plt.show()
    '''
