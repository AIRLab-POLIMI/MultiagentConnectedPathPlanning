'''
Created on Aug 4, 2017

@author: banfi
'''
import cv2
from igraph import *
import matplotlib.pyplot as plt

import gflags

gflags.DEFINE_string('exp_name', 'prova', 'name of the experiment (matching both .exp and .log files)')

def get_graphs(exp_file):
    f = open(exp_file, 'r')
    lines = f.readlines()

    G_E = None
    G_C = None

    for line in lines:
        s = line.split()
        print s
        if s[0] == 'phys_graph':
            phys_graph_path = '../data/' + s[1]
            G_E = read(phys_graph_path, format='graphml')
        elif s[0] == 'comm_graph':
            comm_graph_path = '../data/' + s[1]
            G_C = read(comm_graph_path, format='graphml')

    if G_E is None or G_C is None:
        print 'Error while reading graphs! Aborting.'
        exit(1)

    return G_E,G_C

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

    return plan    

if __name__ == "__main__":
    exp_file = '../data/' + gflags.FLAGS.exp_name + '.exp'
    G_E, G_C = get_graphs(exp_file)

    log_file = '../logs/' + gflags.FLAGS.exp_name + '.log'
    plan = get_plan(log_file)   



