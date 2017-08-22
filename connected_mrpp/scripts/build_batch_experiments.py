'''
Created on Aug 22, 2017

@author: banfi
'''

from igraph import *
import matplotlib.pyplot as plt
import pylab as pl
import os
import random
import time
import sys

import gflags

from utils import get_graphs_and_image_from_files, plot_plan

gflags.DEFINE_string('file_path', '../envs/offices.png', 'png file path')

gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')
gflags.DEFINE_integer('cell_size', 11, 'pixels making 1 grid cell (only for uniform grid discretization)')

gflags.DEFINE_integer('min_range', 50, 'min communication range (in pixels)')
gflags.DEFINE_integer('max_range', 201, 'max communication range (in pixels)')
gflags.DEFINE_integer('step_range', 50, 'communication range step (in pixels)')


gflags.DEFINE_string('output_phys', '../data/offices_phys', 'physical graph output')
gflags.DEFINE_string('output_comm', '../data/offices_comm', 'communication graph output')

gflags.DEFINE_integer('max_robots', 8, 'maximum number of robots')

gflags.DEFINE_integer('n_exp', 2, 'number of experiments')

gflags.DEFINE_integer('min_dist', 200, 'minimum distance to travel between start-goal (in pixels)')

gflags.DEFINE_bool('debug', True, 'debug mode active')

comm_discr_types = ['los','range']

def get_random_connected_config(G_C, size):
    connected = False
    config = random.sample(range(len(G_C.vs)), 1)
    while(len(config) < size):
        neighbor_found = False
        while(not(neighbor_found)):
            candidates = G_C.neighbors(random.choice(config))
            candidates_final = []
            for cand in candidates:
                if cand not in config:
                    candidates_final.append(cand)
            if (len(candidates_final) > 0):
                neighbor_found = True
                config.append(random.choice(candidates_final))

    return config

def write_exp_file(exp_name, phys_graph_file, comm_graph_file, start_config, goal_config):
    f = open('../data/' + exp_name + '.exp', 'w')
    f.write('phys_graph ' + phys_graph_file + '\n')
    f.write('comm_graph ' + comm_graph_file + '\n')
    start_string = ""
    for v in start_config:
        start_string += str(v) + ' '
    goal_string = ""
    for v in goal_config:
        goal_string += str(v) + ' '
    f.write('start ' + start_string + '\n')
    f.write('goal ' + goal_string + '\n')
    f.close()

if __name__ == "__main__":
    argv = gflags.FLAGS(sys.argv)
    random.seed(1)

    for comm_discr_type in comm_discr_types:

        for max_dist in range(gflags.FLAGS.min_range, gflags.FLAGS.max_range, gflags.FLAGS.step_range):
            print 'Range: ', max_dist
            #create graphs file
            os.system('python create_graph_from_png.py --file_path=' + gflags.FLAGS.file_path + \
                      ' --phys_discr_type=' + gflags.FLAGS.phys_discr_type + \
                      ' --cell_size=' + str(gflags.FLAGS.cell_size) + \
                      ' --range=' + str(max_dist) + \
                      ' --output_phys=' + gflags.FLAGS.output_phys + \
                      ' --output_comm=' + gflags.FLAGS.output_comm + \
                      ' --debug=False')

            #read graphs files
            phys_graph_file = gflags.FLAGS.output_phys.split('/')[2] + \
                              '_' + gflags.FLAGS.phys_discr_type + \
                              '_' + str(gflags.FLAGS.cell_size) + \
                              '_' + comm_discr_type + \
                              '_' + str(max_dist) + '.graphml'
            comm_graph_file = gflags.FLAGS.output_comm.split('/')[2] + \
                              '_' + gflags.FLAGS.phys_discr_type + \
                              '_' + str(gflags.FLAGS.cell_size) + \
                              '_' + comm_discr_type + \
                              '_' + str(max_dist) + '.graphml'

            G_E, G_C, im_array = get_graphs_and_image_from_files(phys_graph_file, comm_graph_file)

            shortest_paths = G_E.shortest_paths()

            if(gflags.FLAGS.debug):
                fig = pl.figure()
                ax = fig.add_subplot(111)
                pl.ion()
                pl.plot()
                plot_plan(G_E, G_C, im_array, [], ax)

            for n_robots in range(2, gflags.FLAGS.max_robots + 1):
                print 'N robots: ', n_robots

                for exp in range(gflags.FLAGS.n_exp):
                    print exp
                    #choose random start-goal for n_robots
                    ok = False
                    while(not(ok)):
                        start_config = get_random_connected_config(G_C, n_robots)
                        goal_config = get_random_connected_config(G_C, n_robots)

                        min_distance = min(map(lambda x: shortest_paths[start_config[x]][goal_config[x]], range(n_robots)))

                        if(min_distance > gflags.FLAGS.min_dist/gflags.FLAGS.cell_size): ok = True

                    #write new exp file
                    exp_name = gflags.FLAGS.file_path.split('/')[2].split('.')[0] + \
                               '_' + gflags.FLAGS.phys_discr_type + \
                               '_' + str(gflags.FLAGS.cell_size) + \
                               '_' + comm_discr_type + \
                               '_' + str(max_dist) + \
                               '_' + str(n_robots) + \
                               '_' + str(exp)

                    write_exp_file(exp_name, phys_graph_file, comm_graph_file, start_config, goal_config)
                    print 'Experiment ', exp, ' created'

                    if(gflags.FLAGS.debug):
                        ax.cla()
                        plot_plan(G_E, G_C, im_array, start_config, ax, color='bo')
                        pl.draw()
                        plt.pause(0.0001)
                        time.sleep(1)

                        ax.cla()
                        plot_plan(G_E, G_C, im_array, goal_config, ax, color='ro')
                        pl.draw()
                        plt.pause(0.0001)
                        time.sleep(1)
