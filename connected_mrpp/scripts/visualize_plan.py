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
import sys

from utils import get_graphs_and_image_from_exp, plot_plan

gflags.DEFINE_string('exp_name', 'prova', 'name of the experiment (matching .exp files)')
gflags.DEFINE_string('alg', 'birk', 'name of the algorithm (to select the correletad .log file)')
gflags.DEFINE_integer('speed', 0.5, 'time between two plan steps (in seconds)')

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

if __name__ == "__main__":
    argv = gflags.FLAGS(sys.argv)
    exp_file = '../data/' + gflags.FLAGS.exp_name + '.exp'
    G_E, G_C, im_array = get_graphs_and_image_from_exp(exp_file)

    log_file = '../logs/' + gflags.FLAGS.exp_name + '_' + gflags.FLAGS.alg + '.log'
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
        plt.pause(0.0001)
        time.sleep(gflags.FLAGS.speed)
    pl.close("all")



