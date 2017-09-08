'''

STUFF

'''
import os
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import joypy

df = pd.read_csv("offices_data.csv")

p_range = 100

p_alg = ['RB', 'DFS']

params = {'axes.labelsize': 18,'axes.titlesize':18, 'text.fontsize': 18, 'legend.fontsize': 18, 'xtick.labelsize': 18, 'ytick.labelsize': 18}
matplotlib.rcParams.update(params)

for alg in p_alg :
    df_r = df.loc[df['Range'] == p_range]
    df_ra = df_r.loc[df['Algorithm'] == alg]

    filtered_df = {}
    p_cols = []

    for a in range(2, 11) :
        filtered_df[str(a)] = df_ra.loc[df['Agents'] == a]['Times']
        p_cols.append(str(a))


    #print("\n\n"+alg+"\n\n")
    #print(filtered_df)

    '''joypy.joyplot(filtered_df, column = p_cols, by=None, grid=True,
                xlabelsize=None, xrot=None, ylabelsize=None, yrot=None,
                ax=None, figsize=None,
                hist=False, bins=10,
                fade=False, ylim='max',
                fill=True, linecolor=None,
                overlap=1, background=None,
                labels=None, xlabels=True, ylabels=True,
                range_style='all',
                x_range=None,
                title=None,
                colormap=None,
                bw_method=0.3)'''


sns.boxplot(x = "Agents", y = "Times", hue = "Algorithm", data = df, showfliers = False)
plt.xlabel('Agents')
plt.ylabel('Path Length')

#plt.xlabel('Times')
plt.tight_layout()

plt.savefig('lengths.pdf')
plt.show()