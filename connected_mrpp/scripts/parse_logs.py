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
            results[0] = -1
        else:
            s = line.split()
            if s[0] == 't':
                results[1] = float(s[1])
            elif s[0][0] == 's'
                steps += 1

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
            
            print 'Range: ', max_dist

            for n_robots in range(2, gflags.FLAGS.max_robots + 1):
                data[comm_discr_type][max_dist][n_robots] = {}
                print 'N robots: ', n_robots

                for alg in algorithms:
                    data[comm_discr_type][max_dist][n_robots][alg] = []
                    #print 'Algorithm: ', alg

                    for exp in range(gflags.FLAGS.n_exp):
                        #print 'Experiment: ' + str(exp)
                        log_filepath = os.getcwd() + '/log/' + gflags.FLAGS.env_name + \
                                   '_' + gflags.FLAGS.phys_discr_type + \
                                   '_' + str(gflags.FLAGS.cell_size) + \
                                   '_' + comm_discr_type + \
                                   '_' + str(max_dist) + \
                                   '_' + str(n_robots) + \
                                   '_' + str(exp) + \
                                   '_' + alg + '.exp'

                    results = parse_log(log_filepath)
                    data[comm_discr_type][max_dist][n_robots][alg].append(results)

                #custom for birk and dfs
                solved_birk = 0
                solved_dfs = 0
                winner_feas_birk = 0
                winner_feas_dfs = 0
                winner_birk = 0
                winner_dfs = 0
                times_birk = 0
                times_dfs = 0

                for exp in range(gflags.FLAGS.n_exp):
                    #solved instances
                    if(data[comm_discr_type][max_dist][n_robots]['dfs'][exp][0] > -2):
                        solved_dfs += 1

                    if(data[comm_discr_type][max_dist][n_robots]['birk'][exp][0] > -2):
                        solved_birk += 1

                    #comparison
                    if(data[comm_discr_type][max_dist][n_robots]['dfs'][exp][0] > -2 and 
                       data[comm_discr_type][max_dist][n_robots]['birk'][exp][0] == -2):
                        winner_feas_dfs += 1

                    elif(data[comm_discr_type][max_dist][n_robots]['birk'][exp][0] > -2 and 
                       data[comm_discr_type][max_dist][n_robots]['dfs'][exp][0] == -2):
                        winner_feas_birk +=1

                    elif(data[comm_discr_type][max_dist][n_robots]['dfs'][exp][0] >= 0 and
                        data[comm_discr_type][max_dist][n_robots]['birk'][exp][0] >= 0):

                        if(data[comm_discr_type][max_dist][n_robots]['dfs'][exp][0] < 
                           data[comm_discr_type][max_dist][n_robots]['birk'][exp][0):
                            winner_dfs += 1

                        elif(data[comm_discr_type][max_dist][n_robots]['birk'][exp][0] < 
                           data[comm_discr_type][max_dist][n_robots]['dfs'][exp][0):
                            winner_birk += 1

                    #times
                    if(data[comm_discr_type][max_dist][n_robots]['dfs'][exp][0] > -2):
                        times_dfs.append(data[comm_discr_type][max_dist][n_robots]['dfs'][exp][1]

                    elif(data[comm_discr_type][max_dist][n_robots]['birk'][exp][0] > -2):
                        times_birk.append(data[comm_discr_type][max_dist][n_robots]['birk'][exp][1]

                print 'Solved instances birk: ', solved_birk
                print 'Solved instances dfs: ', solved_dfs
                
                print 'Winner feas/unfeas birk: ', winner_feas_birk
                print 'Winner feas/unfeas dfs: ', winner_feas_dfs
                
                print 'Winner time-optimal birk: ', winner_birk
                print 'Winner time-optimal dfs: ', winner_dfs

                print 'Avg time birk', sum(times_birk)/len(times_birk)
                print 'Avg time dfs', sum(times_dfs)/len(times_dfs)