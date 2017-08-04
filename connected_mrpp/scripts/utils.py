'''
Created on Aug 4, 2017

@author: banfi
'''

from igraph import *
import cv2

def get_graphs_and_image_from_files(phys_graph_file, comm_graph_file):
    G_E = None
    G_C = None
    im_array = None

    phys_graph_path = '../data/' + phys_graph_file
    G_E = read(phys_graph_path, format='graphml')

    image_path = '../envs/' + phys_graph_file.split('_')[0] + '.png'
    im = cv2.imread(image_path)
    im_array = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

    comm_graph_path = '../data/' + comm_graph_file
    G_C = read(comm_graph_path, format='graphml')

    if G_E is None or G_C is None or im_array is None:
        print 'Error while reading graphs! Aborting.'
        exit(1)

    return G_E, G_C, im_array

def get_graphs_and_image_from_exp(exp_file):
    f = open(exp_file, 'r')
    lines = f.readlines()

    G_E = None
    G_C = None
    im_array = None

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

    if G_E is None or G_C is None or im_array is None:
        print 'Error while reading graphs! Aborting.'
        exit(1)

    return G_E, G_C, im_array