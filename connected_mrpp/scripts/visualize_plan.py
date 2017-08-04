'''
Created on Aug 4, 2017

@author: banfi
'''
import cv2
from igraph import *
import matplotlib.pyplot as plt

import gflags

gflags.DEFINE_string('exp_name', 'prova', 'name of the experiment (matching both .exp and .log files)')

if __name__ == "__main__":
    exp_file = '../data/' + gflags.FLAGS.exp_name + '.exp'
    log_file = '../logs/' + gflags.FLAGS.exp_name + '.log'

    print log_file



