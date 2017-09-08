'''

STUFF

'''
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import joypy

df = pd.read_csv("offices_data.csv")

p_range = 100

p_alg = ['birk', 'dfs', 'astar']

for alg in p_alg :
    df_r = df.loc[df['range'] == p_range]
    df_ra = df_r.loc[df['alg'] == alg]

    filtered_df = {}
    p_cols = []

    for a in range(2, 11) :
        filtered_df[str(a)] = df_ra.loc[df['agents'] == a]['times']
        p_cols.append(str(a))

    joypy.joyplot(filtered_df, column = p_cols, by=None, grid=True,
                xlabelsize=None, xrot=None, ylabelsize=None, yrot=None,
                ax=None, figsize=None,
                hist=False, bins=10,
                fade=False, ylim='max',
                fill=True, linecolor=None,
                overlap=1.5, background=None,
                labels=None, xlabels=True, ylabels=True,
                range_style='all',
                x_range=None,
                title=None,
                colormap=None,
                bw_method=0.2)

plt.xlabel('Times')

plt.show()