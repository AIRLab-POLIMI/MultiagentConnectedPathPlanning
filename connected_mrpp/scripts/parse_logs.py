'''
Created on Aug 22, 2017

@author: banfi
'''
import os
import sys
import gflags
from posixfile import _posixfile_

from PIL.ImageOps import solarize

gflags.DEFINE_string('env_name', 'offices', 'environment name')

gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')
gflags.DEFINE_integer('cell_size', 11, 'pixels making 1 grid cell (only for uniform grid discretization)')

gflags.DEFINE_integer('min_range', 50, 'min communication range (in pixels)')
gflags.DEFINE_integer('max_range', 201, 'max communication range (in pixels)')
gflags.DEFINE_integer('step_range', 50, 'communication range step (in pixels)')

gflags.DEFINE_integer('max_robots', 10, 'maximum number of robots')

gflags.DEFINE_integer('n_exp', 50, 'number of experiments')

gflags.DEFINE_string('deadline', '60', 'deadline (in seconds)')
gflags.DEFINE_string('log_subfolder', '2017-08-25_13-22-26', 'the log subfolder to parse')

comm_discr_types = ['range']
algorithms = ['birk', 'dfs', 'astar']

def parse_log(log_filepath):
    results = [None, None]
    f = open(log_filepath, 'r')
    lines = f.readlines()
    steps = 0
    for line in lines:
        if 'NO_PLAN_FOUND_WITHIN_DEADLINE' in line:
            results = [-2, int(gflags.FLAGS.deadline)]
            return results
        elif 'NO_PLAN_EXISTS' in line:
            print "NO PLAN CONFIRMED FOUND IN ", log_filepath
            results[0] = -1
        else:
            s = line.split()
            if s[0] == 't':
                results[1] = float(s[1])
            elif s[0][0] == 's':
                steps += 1

    if results[0] != -1:
        results[0] = steps

    return results

if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    argv = gflags.FLAGS(sys.argv)

    print 'Parsing experiments on environment ', gflags.FLAGS.env_name

    data = {}

    for comm_discr_type in comm_discr_types:
        print 'Comm discr type: ', comm_discr_type
        data[comm_discr_type] = {}

        for max_dist in range(gflags.FLAGS.min_range, gflags.FLAGS.max_range, gflags.FLAGS.step_range):
            data[comm_discr_type][max_dist] = {}
            
            print '\tRange: ', max_dist

            for n_robots in range(2, gflags.FLAGS.max_robots + 1):
                data[comm_discr_type][max_dist][n_robots] = {}
                print '\t\tN robots: ', n_robots

                for alg in algorithms:
                    data[comm_discr_type][max_dist][n_robots][alg] = []
                    #print 'Algorithm: ', alg

                    for exp in range(gflags.FLAGS.n_exp):
                        #print 'Experiment: ' + str(exp)
                        log_filepath = os.getcwd() + '/logs/' + gflags.FLAGS.log_subfolder + \
                                   '/' + gflags.FLAGS.env_name + \
                                   '_' + gflags.FLAGS.phys_discr_type + \
                                   '_' + str(gflags.FLAGS.cell_size) + \
                                   '_' + comm_discr_type + \
                                   '_' + str(max_dist) + \
                                   '_' + str(n_robots) + \
                                   '_' + str(exp) + \
                                   '_' + alg + '.log'

                        results = parse_log(log_filepath)
                        data[comm_discr_type][max_dist][n_robots][alg].append(results)

                #prepare data
                p_dict = dict()
                for alg in algorithms:
                    p = dict(solved=0, winner=0, time_opt=0, times=[])
                    p_dict[alg] = p

                for exp in range(gflags.FLAGS.n_exp):
                    #solved instances
                    for alg in algorithms:
                        if data[comm_discr_type][max_dist][n_robots][alg][exp][0] > -2:
                            p_dict[alg]['solved'] += 1

                    #comparison
                    n_failed = 0
                    for alg in algorithms:
                        if data[comm_discr_type][max_dist][n_robots][alg][exp][0] == -2:
                            n_failed += 1

                    if n_failed == len(algorithms) - 1:
                        for alg in algorithms:
                            if data[comm_discr_type][max_dist][n_robots][alg][exp][0] > -2:
                                p_dict[alg]['winner'] += 1
                                break


                    #Solution time
                    best = []
                    for alg in algorithms:
                        if data[comm_discr_type][max_dist][n_robots][alg][exp][0] > -2:
                            if len(best) == 0:
                                best = [alg]
                            elif (data[comm_discr_type][max_dist][n_robots][alg][exp][0] > \
                                data[comm_discr_type][max_dist][n_robots][best[0]][exp][0]):
                                best = [alg]
                            elif (data[comm_discr_type][max_dist][n_robots][alg][exp][0] == \
                                data[comm_discr_type][max_dist][n_robots][best[0]][exp][0]):
                                best.append(alg)
                                

                    if len(best) > 0:
                        for alg in best:
                            p_dict[alg]['time_opt'] += 1

                    #times
                    for alg in algorithms:
                        if data[comm_discr_type][max_dist][n_robots][alg][exp][0] > -2:
                            p_dict[alg]['times'].append(data[comm_discr_type][max_dist][n_robots][alg][exp][1])

                #print data
                for alg in algorithms:
                    print '\t\tSolved instances ', alg, ': ', p_dict[alg]['solved']

                for alg in algorithms:
                    print '\t\tWinner feas/unfeas ', alg, ': ', p_dict[alg]['winner']
                
                for alg in algorithms:
                    print '\t\tWinner time-optimal ', alg, ': ', p_dict[alg]['time_opt']

                for alg in algorithms:
                    times = p_dict[alg]['times']
                    if(len(times)):
                        print '\t\tAvg time ', alg, ': ', sum(times)/len(times)
                    else:
                        print '\t\t', alg, 'did not find any feasible plan.'