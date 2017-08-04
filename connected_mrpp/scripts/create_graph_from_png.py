'''
Created on Jul 31, 2017

@author: banfi
'''
import math
import cv2
import numpy as np
from igraph import *
import matplotlib.pyplot as plt

import gflags

gflags.DEFINE_string('file_path', '../envs/offices.png', 'png file path')

gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')
gflags.DEFINE_integer('cell_size', 11, 'pixels making 1 grid cell (only for uniform grid discretization)')

gflags.DEFINE_string('comm_discr_type', 'range', 'environment discretization - comm')
gflags.DEFINE_integer('range', 150, 'communication range (in pixels)')

gflags.DEFINE_string('output_phys', '../data/offices_phys', 'physical graph output')
gflags.DEFINE_string('output_comm', '../data/offices_comm', 'communication graph output')

gflags.DEFINE_bool('debug', True, 'debug mode active')

def is_grid_cell(im_array, i, j, rows, cols):
    for k in range(i, i + gflags.FLAGS.cell_size):
        if k >= rows: return False

        for w in range(j, j + gflags.FLAGS.cell_size):
            if w >= cols: return False

            if im_array[k][w] == 0: return False

    return True

def create_phys_graph_grid():
    print 'Creating grid physical graph...'
    im = cv2.imread(gflags.FLAGS.file_path)
    im_array = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    
    rows = np.size(im_array,0)
    cols = np.size(im_array,1)

    G_E = Graph(directed=False)
    curr_id = 0
    for i in range(0, rows, gflags.FLAGS.cell_size):
        for j in range(0, cols, gflags.FLAGS.cell_size):
            if is_grid_cell(im_array, i,j, rows, cols):
                G_E.add_vertex()
                G_E.vs[curr_id]['x_coord'] = i
                G_E.vs[curr_id]['y_coord'] = j
                curr_id += 1

    neighbors = []
    for vertex_id in range(curr_id):
        
        curr_x = G_E.vs[vertex_id]['x_coord']
        curr_y = G_E.vs[vertex_id]['y_coord']

        up = G_E.vs.select(x_coord_eq=curr_x, y_coord_eq=(curr_y + gflags.FLAGS.cell_size))
        if len(up):
            neighbors.append((vertex_id, up[0].index))

        right = G_E.vs.select(x_coord_eq=(curr_x + gflags.FLAGS.cell_size), y_coord_eq=curr_y)
        if len(right):
            neighbors.append((vertex_id, right[0].index))

    G_E.add_edges(neighbors)
    print 'Done. Number of vertices: ', len(G_E.vs)

    if gflags.FLAGS.debug:
        plt.imshow(im_array)
        bu = gflags.FLAGS.cell_size / 2
        for edge in G_E.es:
            v1 = G_E.vs[edge.source]
            v2 = G_E.vs[edge.target]
            plt.plot([v1['y_coord'] + bu ,v2['y_coord'] + bu],[v1['x_coord'] + bu, v2['x_coord'] + bu],'b')

        plt.show()

    return G_E,im_array

def create_comm_graph_range(G_E, im_array=None):
    print 'Creating range communication graph...'
    G_C = Graph(directed=False)
    for vertex_id in range(len(G_E.vs)):
        G_C.add_vertex()
        G_C.vs[vertex_id]['x_coord'] = G_E.vs[vertex_id]['x_coord']
        G_C.vs[vertex_id]['y_coord'] = G_E.vs[vertex_id]['y_coord']

    neighbors = []
    range_squared = gflags.FLAGS.range**2

    for vertex_id_1 in range(len(G_C.vs)):
        for vertex_id_2 in range(vertex_id_1 + 1, len(G_C.vs)):
            if (G_C.vs[vertex_id_1]['x_coord']- G_C.vs[vertex_id_2]['x_coord'])**2 + \
               (G_C.vs[vertex_id_1]['y_coord']- G_C.vs[vertex_id_2]['y_coord'])**2 <= range_squared:
               neighbors.append((vertex_id_1, vertex_id_2))

    G_C.add_edges(neighbors)

    print 'Done.'

    if gflags.FLAGS.debug:
        plt.imshow(im_array)
        bu = gflags.FLAGS.cell_size / 2 + 1
        for edge in G_C.es.select(_source=0):
            v1 = G_C.vs[edge.source]
            v2 = G_C.vs[edge.target]
            plt.plot([v1['y_coord'] + bu ,v2['y_coord'] + bu],[v1['x_coord'] + bu, v2['x_coord'] + bu],'b')
        for edge in G_C.es.select(_target=0):
            v1 = G_C.vs[edge.source]
            v2 = G_C.vs[edge.target]
            plt.plot([v1['y_coord'] + bu ,v2['y_coord'] + bu],[v1['x_coord'] + bu, v2['x_coord'] + bu],'b')

        plt.show()
    return G_C

if __name__ == "__main__":
    if gflags.FLAGS.phys_discr_type == 'uniform_grid':
        G_E, im_array = create_phys_graph_grid()
        G_E.write(gflags.FLAGS.output_phys + '_' + gflags.FLAGS.phys_discr_type + '_' + \
                  str(gflags.FLAGS.cell_size) + '_' + gflags.FLAGS.comm_discr_type + '_' + \
                  str(gflags.FLAGS.range) + '.graphml', format='graphml')
    else:
        print 'Error! Physical discretization not supported!'
        exit(1)

    if gflags.FLAGS.comm_discr_type == 'range':
        G_C = create_comm_graph_range(G_E, im_array)
        G_C.write(gflags.FLAGS.output_comm + '_' + gflags.FLAGS.phys_discr_type + '_' + \
                  str(gflags.FLAGS.cell_size) + '_' + gflags.FLAGS.comm_discr_type + '_' + \
                  str(gflags.FLAGS.range) + '.graphml', format='graphml')
    else:
        print 'Error! Comm discretization not supported!'
        exit(1)



