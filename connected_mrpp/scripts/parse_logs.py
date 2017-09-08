'''
Created on Aug 22, 2017

@author: banfi
'''
import os
import sys
import gflags
import matplotlib
import numpy as np
import seaborn as sns
import pandas as pd

from posixfile import _posixfile_

from PIL.ImageOps import solarize

from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


gflags.DEFINE_string('env_name', 'offices', 'environment name')

gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')
gflags.DEFINE_integer('cell_size', 11, 'pixels making 1 grid cell (only for uniform grid discretization)')

gflags.DEFINE_integer('min_range', 50, 'min communication range (in pixels)')
gflags.DEFINE_integer('max_range', 201, 'max communication range (in pixels)')
gflags.DEFINE_integer('step_range', 50, 'communication range step (in pixels)')

gflags.DEFINE_integer('max_robots', 10, 'maximum number of robots')

gflags.DEFINE_integer('n_exp', 50, 'number of experiments')

gflags.DEFINE_string('deadline', '300.0', 'deadline (in seconds)')
gflags.DEFINE_string('log_subfolder', '2017-08-25_13-22-26', 'the log subfolder to parse')

gflags.DEFINE_string('show_delta', 'false', 'show delta times')

gflags.DEFINE_string('charts_dir', 'charts/', 'folder containing visual results')

gflags.DEFINE_string('latex_tab', 'latex_tab.tex', 'file containing latex-formatted results')


comm_discr_types = ['range']
algorithms = ['birk', 'dfs', 'astar']


def joyplot(data, ax=None, 
            flatten = .1, #rescale the height of each distribution to avoid overlap. If large, will flatten out each of the KDEs
            linecolor='k', 
            shadecolor='w',
            alpha=1,
            shade=True, 
            line_kws = None,
            kde_kws=None,
            fig_kws=None,
            shade_kws=None):
    line_kws = dict() if line_kws is None else line_kws
    kde_kws = (dict(kernel='gau', bw='scott',
                         gridsize=100, cut=3,
                         clip=None) if kde_kws is None else kde_kws)
    fig_kws = dict(figsize=(5,5)) if fig_kws is None else fig_kws
    shade_kws = (dict(alpha=alpha, 
                          clip_on=True, 
                          color=shadecolor) if shade_kws is None else shade_kws)
    if kde_kws.get('clip',None) is None:
        kde_kws['clip'] = (-np.inf, np.inf)
    if ax is None:
        f,ax = plt.subplots(1,1, **fig_kws)
    T,N = data.shape
    dsupport = np.array([])
    max_zorder = 2*T
    zorder = max_zorder
    for i, row in enumerate(data):
        x,y = _statsmodels_univariate_kde(row, **kde_kws)
        y = np.max(np.c_[np.zeros_like(y), y], axis=1)
        y = y/(flatten*y.max()) + i
        ax.plot(x,y,color=linecolor, zorder=zorder, **line_kws)
        if shade:
            if shade_kws.get('color', None) is None:
                shade_kws['color'] = shadecolor
            ax.fill_between(x, i, y, zorder=zorder-1,
                             **shade_kws)
        dsupport = np.concatenate((dsupport, x))
        zorder -= 2
    ax.set_xlim(np.min(dsupport)*.75, np.max(dsupport)*1.25)
    return f,ax


def parse_log(log_filepath):
    results = [None, None]
    f = open(log_filepath, 'r')
    lines = f.readlines()
    steps = 0
    for line in lines:
        if 'NO_PLAN_FOUND_WITHIN_DEADLINE' in line:
            results = [-2, float(gflags.FLAGS.deadline)]
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

    sol_found = dict()
    avg_time = dict()
    for r in xrange(2, gflags.FLAGS.max_robots + 1) :
        avg_time[r] = dict()
        sol_found[r] = dict()
        for c in [25] + range(gflags.FLAGS.min_range, gflags.FLAGS.max_range, gflags.FLAGS.step_range) :
            avg_time[r][c] = dict()
            sol_found[r][c] = dict()


    for comm_discr_type in comm_discr_types:
        print 'Comm discr type: ', comm_discr_type
        data[comm_discr_type] = {}

        for max_dist in [25] + range(gflags.FLAGS.min_range, gflags.FLAGS.max_range, gflags.FLAGS.step_range):
            data[comm_discr_type][max_dist] = {}
            
            print '\tRange: ', max_dist

            for n_robots in xrange(2, gflags.FLAGS.max_robots + 1):
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
                at_least_one_count = 0
                for alg in algorithms:
                    p = dict(solved=0, winner=0, time_opt=0, times=[])
                    p_dict[alg] = p

                for exp in range(gflags.FLAGS.n_exp):
                    #solved instances
                    feasibility_found = False
                    for alg in algorithms:
                        if data[comm_discr_type][max_dist][n_robots][alg][exp][0] > -2:
                            if not(feasibility_found):
                                feasibility_found = True
                                at_least_one_count += 1

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
                        if data[comm_discr_type][max_dist][n_robots][alg][exp][0] > -2 :
                            p_dict[alg]['times'].append(data[comm_discr_type][max_dist][n_robots][alg][exp][1])
                        elif gflags.FLAGS.show_delta == 'true'  :
                            p_dict[alg]['times'].append(float(gflags.FLAGS.deadline))
                        else :
                            p_dict[alg]['times'].append(float(gflags.FLAGS.deadline))

                #print data
                print '\t\tInstances solved (feas.) by at least one algorithm: ', at_least_one_count
                sol_found[n_robots][max_dist]['tot'] = at_least_one_count

                for alg in algorithms:
                    print '\t\tSolved instances ', alg, ': ', p_dict[alg]['solved']
                    sol_found[n_robots][max_dist][alg] = p_dict[alg]['solved']

                for alg in algorithms:
                    print '\t\tWinner feas/unfeas ', alg, ': ', p_dict[alg]['winner']
                
                for alg in algorithms:
                    print '\t\tWinner time-optimal ', alg, ': ', p_dict[alg]['time_opt']

                

                min_times = []

                for alg in algorithms:
                    times = p_dict[alg]['times']
                    if(len(times)):
                        running_time = sum(times)/len(times)
                        print '\t\tAvg time ', alg, ': ', running_time
                    else:
                        print '\t\t', alg, 'did not find any feasible plan.'

                    for j in xrange(len(p_dict[alg]['times'])) :
                        if len(min_times) <= j :
                            min_times.append(p_dict[alg]['times'][j])
                        elif min_times[j] > p_dict[alg]['times'][j] :
                            min_times[j] = p_dict[alg]['times'][j]

                for alg in algorithms:
                    if gflags.FLAGS.show_delta == 'true' :
                        avg_time[n_robots][max_dist][alg] = [p_dict[alg]['times'][j] - min_times[j] for j in xrange(len(p_dict[alg]['times']))]
                    else :
                        avg_time[n_robots][max_dist][alg] = p_dict[alg]['times']

    ###### Create Boxplots ######

    '''print '\n\t> Creating Boxplot...\n'

    matplotlib.rcParams.update({'font.size': 14})
    fig = plt.figure()

    line_markers = {algorithms[0]: 'o', algorithms[1]: '^', algorithms[2]: 'v'}
    line_styles = {algorithms[0]: '--', algorithms[1]: '-', algorithms[2]: ':'}
    for alg in algorithms:
        plt.boxplot(Y_ax[alg])

    plt.legend(algorithms)
    plt.xlabel('Agents')
    plt.ylabel('Solved')
    plt.ylim(0, 50)
    plt.xticks(np.arange(2, gflags.FLAGS.max_robots + 1, 1.0))

    pdf.savefig(fig)'''

    comm_radii = [50, 100, 150]

    # Initialize the figure with a logarithmic x axis
    f, ax = plt.subplots(figsize=(7, 6))
    #ax.set_yscale("log")

    t_dataset = []
    n_dataset = []
    a_dataset = []
    c_dataset = []

    for r in xrange(2, gflags.FLAGS.max_robots + 1) :
        for c in comm_radii :
            for alg in algorithms :
                t_dataset = t_dataset + avg_time[r][c][alg]
                n_dataset = n_dataset + [r for j in avg_time[r][c][alg]]
                a_dataset = a_dataset + [alg for j in avg_time[r][c][alg]]
                c_dataset = c_dataset + [c for j in avg_time[r][c][alg]]

    #dset = {'times' : pd.Series(t_dataset), 'agents' : pd.Series(n_dataset), 'alg': pd.Series(a_dataset)}

    # Draw a nested boxplot to show bills by day and sex
    #sns.boxplot(x = "agents", y = "times", hue = "alg", data = dset, showfliers = False)
    #sns.tsplot(time = "agents", value = "times", unit = "subject", condition = "alg", data = dset)
    #sns.lmplot(x = "agents", y = "times", col = "alg", hue = "alg", data=dset, y_jitter=.02, logistic=True)
    #sns.factorplot(x = "agents", y = "times", hue = "alg", data=dset)


    csv_file = open(gflags.FLAGS.env_name + "_data.csv", 'w+')

    csv_file.write("times,agents,alg,range\n")

    for i in xrange(len(t_dataset)) :
        csv_file.write(str(t_dataset[i]) + ",")
        csv_file.write(str(n_dataset[i]) + ",")
        csv_file.write(str(a_dataset[i]) + ",")
        csv_file.write(str(c_dataset[i]) + "\n")

    #plt.show()


    ###### Create a Latex File ######

    print '\t> Creating Latex Table...'

    latex_file = open(gflags.FLAGS.charts_dir + gflags.FLAGS.env_name
        + '_' + gflags.FLAGS.latex_tab, 'w+')

    # Writing head   
    latex_file.write('\\begin{tabular}{|c')
    
    for c in comm_radii :
        for alg in algorithms :
            latex_file.write('|c')

    latex_file.write('|}\n\\hline\n')

    for c in comm_radii :
        latex_file.write('& \\multicolumn{' + str(len(algorithms) + 1) + '}{|c|}{Range: ' + str(c) + 'px} ')

    latex_file.write('\\\\ \\hline\n')

    latex_file.write('\\# Robots & ')

    for c in comm_radii :
        for alg in algorithms :
            latex_file.write(alg + ' ')
            fpos = latex_file.tell()
            latex_file.write('& ')

    latex_file.seek(fpos)
    latex_file.write('\\\\ \\hline\n')


    # Writing body
    for r in xrange(2, gflags.FLAGS.max_robots + 1) :
        latex_file.write(str(r) + ' & ')
        for c in comm_radii :
            for alg in algorithms :
                tab_cell = str(int(sol_found[r][c][alg]))
                if sol_found[r][c][alg] == sorted(sol_found[r][c].values(), reverse=True)[1] :
                    latex_file.write('\\textbf{' + tab_cell + '} ')
                else :
                    latex_file.write(tab_cell + ' ')
                latex_file.write('& ')

            latex_file.write(str(sol_found[r][c]['tot']) + ' ')
            if c != comm_radii[-1] :
                latex_file.write('& ')

        latex_file.write('\\\\ \\hline\n')

    latex_file.write('\\end{tabular}')