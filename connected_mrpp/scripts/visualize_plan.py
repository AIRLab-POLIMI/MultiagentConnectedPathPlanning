'''
Created on Aug 4, 2017

@author: banfi
'''
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pylab as pl
import time
import gflags

from utils import get_graphs_and_image_from_exp

gflags.DEFINE_string('exp_name', 'prova', 'name of the experiment (matching both .exp and .log files)')

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

def plot_plan(G_E, G_C, im_array, config, ax):
    ax.imshow(im_array, cmap=cm.Greys_r)

    for robot in range(len(config)):
        ax.plot([G_E.vs[config[robot]]['x_coord']],[G_E.vs[config[robot]]['y_coord']], 'bo', markersize = 16)
        ax.annotate(str(robot), xy=(G_E.vs[config[robot]]['x_coord'], G_E.vs[config[robot]]['y_coord']), 
                    xytext=(G_E.vs[config[robot]]['x_coord'] - 6, G_E.vs[config[robot]]['y_coord'] + 8), color='w')

    for robot1 in range(len(config)):
        for robot2 in range(robot1 + 1, len(config)):
            if G_C.are_connected(G_E.vs[config[robot1]].index, G_E.vs[config[robot2]].index):
                ax.plot([G_E.vs[config[robot1]]['x_coord'], G_E.vs[config[robot2]]['x_coord']], 
                        [G_E.vs[config[robot1]]['y_coord'], G_E.vs[config[robot2]]['y_coord']], 'g')

    ax.set_xlim([0, np.size(im_array,1)])
    ax.set_ylim([np.size(im_array,0), 0])

if __name__ == "__main__":
    exp_file = '../data/' + gflags.FLAGS.exp_name + '.exp'
    G_E, G_C, im_array = get_graphs_and_image_from_exp(exp_file)

    log_file = '../logs/' + gflags.FLAGS.exp_name + '.log'
    plan = get_plan(log_file)

    fig = pl.figure()
    ax = fig.add_subplot(111)
    pl.ion()
    pl.plot()
    plot_plan(G_E, G_C, im_array, [], ax)

    for step in range(len(plan)):
        ax.cla()
        plot_plan(G_E, G_C, im_array, plan[step], ax)
        pl.draw()
        time.sleep(1)

    pl.close("all")



