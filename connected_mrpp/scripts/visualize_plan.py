'''
Created on Aug 4, 2017

@author: banfi
'''
import cv2
import numpy as np
from igraph import *
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pylab as pl
import time
import gflags

gflags.DEFINE_string('exp_name', 'prova', 'name of the experiment (matching both .exp and .log files)')

def get_graphs_and_image(exp_file):
    f = open(exp_file, 'r')
    lines = f.readlines()

    G_E = None
    G_C = None

    for line in lines:
        s = line.split()
        if s[0] == 'phys_graph':
            phys_graph_path = '../data/' + s[1]
            G_E = read(phys_graph_path, format='graphml')

            image_path = '../envs/' + s[1].split('_')[0] + '.png'
            im = cv2.imread(image_path)
            im_array = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        elif s[0] == 'comm_graph':
            comm_graph_path = '../data/' + s[1]
            G_C = read(comm_graph_path, format='graphml')

    f.close()

    if G_E is None or G_C is None:
        print 'Error while reading graphs! Aborting.'
        exit(1)

    return G_E, G_C, im_array

def get_plan(log_file):
    """
        For now, assumes that configurations are sorted by increasing step
    """
    f = open(log_file)
    lines = f.readlines()
    
    plan = [] 

    for line in lines:
        s = line.split()
        plan.append(map(lambda x: int(x), s[1:]))

    f.close()

    return plan

def plot_plan(G_E, G_C, im_array, config):
    ax.imshow(im_array, cmap=cm.Greys_r)

    for robot in range(len(config)):
        ax.plot([G_E.vs[config[robot]]['y_coord']],[G_E.vs[config[robot]]['x_coord']], 'bo', markersize = 16)
        ax.annotate(str(robot), xy=(G_E.vs[config[robot]]['y_coord'], G_E.vs[config[robot]]['x_coord']), 
                    xytext=(G_E.vs[config[robot]]['y_coord'] - 6, G_E.vs[config[robot]]['x_coord'] + 8), color='w')

    for robot1 in range(len(config)):
        for robot2 in range(robot1 + 1, len(config)):
            if G_C.are_connected(robot1, robot2):
                ax.plot([G_E.vs[config[robot1]]['y_coord'], G_E.vs[config[robot2]]['y_coord']], 
                        [G_E.vs[config[robot1]]['x_coord'], G_E.vs[config[robot2]]['x_coord']], 'g')

    ax.set_xlim([0, np.size(im_array,1)])
    ax.set_ylim([np.size(im_array,0), 0])

if __name__ == "__main__":
    exp_file = '../data/' + gflags.FLAGS.exp_name + '.exp'
    G_E, G_C, im_array = get_graphs_and_image(exp_file)

    log_file = '../logs/' + gflags.FLAGS.exp_name + '.log'
    plan = get_plan(log_file)

    fig = pl.figure()
    ax = fig.add_subplot(111)
    pl.ion()
    pl.plot()
    plot_plan(G_E, G_C, im_array, [])

    for step in range(len(plan)):
        ax.cla()
        plot_plan(G_E, G_C, im_array, plan[step])
        pl.draw()
        time.sleep(1)

    pl.close("all")



