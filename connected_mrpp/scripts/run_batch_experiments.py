'''
Created on Aug 22, 2017

@author: banfi
'''
import os
import sys
import gflags


gflags.DEFINE_string('env_name', 'offices', 'environment name')

gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')
gflags.DEFINE_integer('cell_size', 11, 'pixels making 1 grid cell (only for uniform grid discretization)')

gflags.DEFINE_integer('min_range', 50, 'min communication range (in pixels)')
gflags.DEFINE_integer('max_range', 201, 'max communication range (in pixels)')
gflags.DEFINE_integer('step_range', 50, 'communication range step (in pixels)')

gflags.DEFINE_integer('max_robots', 10, 'maximum number of robots')

gflags.DEFINE_integer('n_exp', 50, 'number of experiments')

gflags.DEFINE_string('deadline', '60', 'deadline (in seconds)')

comm_discr_types = ['range']
algorithms = ['birk', 'dfs']

if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    argv = gflags.FLAGS(sys.argv)

    print 'Running experiments on environment ', gflags.FLAGS.env_name

    for comm_discr_type in comm_discr_types:
        print 'Comm discr type: ', comm_discr_type

        for max_dist in range(gflags.FLAGS.min_range, gflags.FLAGS.max_range, gflags.FLAGS.step_range):
            print 'Range: ', max_dist

            for n_robots in range(2, gflags.FLAGS.max_robots + 1):
                print 'N robots: ', n_robots

                for exp in range(gflags.FLAGS.n_exp):
                    print 'Experiment: ' + str(exp)
                    exp_name = os.getcwd() + '/data/' + gflags.FLAGS.env_name + \
                               '_' + gflags.FLAGS.phys_discr_type + \
                               '_' + str(gflags.FLAGS.cell_size) + \
                               '_' + comm_discr_type + \
                               '_' + str(max_dist) + \
                               '_' + str(n_robots) + \
                               '_' + str(exp) + '.exp'

                    for alg in algorithms:
                        print 'Algorithm: ', alg
                        os.system('rosrun connected_mrpp cmrpp_test ' + exp_name + ' ' + alg + ' ' + gflags.FLAGS.deadline)
