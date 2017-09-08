'''
Created on Aug 22, 2017

@author: banfi
'''
import os
import sys
import gflags
import datetime

from joblib import Parallel, delayed


gflags.DEFINE_string('env_name', 'offices', 'environment name')

gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')
gflags.DEFINE_integer('cell_size', 11, 'pixels making 1 grid cell (only for uniform grid discretization)')

gflags.DEFINE_integer('min_range', 50, 'min communication range (in pixels)')
gflags.DEFINE_integer('max_range', 201, 'max communication range (in pixels)')
gflags.DEFINE_integer('step_range', 50, 'communication range step (in pixels)')

gflags.DEFINE_integer('max_robots', 10, 'maximum number of robots')

gflags.DEFINE_integer('n_exp', 50, 'number of experiments')

gflags.DEFINE_string('deadline', '60', 'deadline (in seconds)')

gflags.DEFINE_integer('n_jobs', -1, 'number of parallel experiments')

comm_discr_types = ['range']
#algorithms = ['birk', 'dfs', 'astar']
#algorithms = ['birk']
algorithms = ['astar']
algoritms_params = {'birk': 'null sum_shortest_path', 'dfs': 'zero sum_shortest_path', 'astar': 'epsilon_distance sum_shortest_path'}

if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    argv = gflags.FLAGS(sys.argv)

    print 'Running experiments on environment ', gflags.FLAGS.env_name

    subdir = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    mydir = os.path.join(os.getcwd(), 'logs', subdir)
    os.mkdir(mydir)
    Parallel(n_jobs=gflags.FLAGS.n_jobs)(delayed(os.system)
                                        ('rosrun connected_mrpp cmrpp_test ' + os.getcwd() + 
                                         '/data/' + gflags.FLAGS.env_name +
                                         '_' + gflags.FLAGS.phys_discr_type + 
                                         '_' + str(gflags.FLAGS.cell_size) + 
                                         '_' + comm_discr_type + 
                                         '_' + str(max_dist) + 
                                         '_' + str(n_robots) + 
                                         '_' + str(exp) + '.exp' + 
                                         ' ' + alg +
                                         ' ' + algoritms_params[alg] +
                                         ' ' + gflags.FLAGS.deadline +
                                         ' ' + subdir)
                                         for alg in algorithms
                                         for exp in range(gflags.FLAGS.n_exp)
                                         for n_robots in range(2, gflags.FLAGS.max_robots + 1)
                                         for max_dist in range(gflags.FLAGS.min_range, gflags.FLAGS.max_range, gflags.FLAGS.step_range)
                                         for comm_discr_type in comm_discr_types)
    
