import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Read the data from the csv file
def read_csv(file):
    df = pd.read_csv(file)

    # access the cell in the last row and "flowtime_improvement_perc" column
    flowtime_improvement_perc = df.iloc[-2]['flowtime_improvement_perc']
    makespan_improvement_perc = df.iloc[-2]['makespan_improvement_perc']
    flow_std = df.iloc[-1]['flowtime_improvement_perc']
    ms_std = df.iloc[-1]['makespan_improvement_perc']
    return flowtime_improvement_perc, makespan_improvement_perc, flow_std, ms_std

# Plot the data
envs = ['dual_gp4', 'panda_two', 'panda_three', 'panda_two_rod']
t_values = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]
tight_values = ['tight', 'loose']
colors = ['r', 'g']

plt.figure()
for env in envs:
    plt.subplot(2, 2, envs.index(env) + 1)
    plt.title(f'{env}')
    for color, tight in zip(colors, tight_values):
        vals = []
        stds = []
        for t in t_values:
            # Read the data
            dir = f'../outputs/t={t}_{tight}'
            flowtime_improvement_perc, makespan_improvement_perc, flow_std, ms_std = read_csv(f'{dir}/{env}_benchmark.avg.csv')
            vals.append(makespan_improvement_perc)
            stds.append(ms_std)

        
        # Plot the data with the specified color and label
        plt.scatter(t_values, vals, label=f'{tight}', marker='x', color=color)

        # Plot the stddev as shaded region with the same color but without a label
        plt.fill_between(t_values, np.array(vals) - np.array(stds), np.array(vals) + np.array(stds), alpha=0.2, color=color)

    # log axis for x
    plt.xscale('log')
    plt.xlabel('Time (s)')
    plt.ylabel('Perct Improvement (%)')

plt.legend()
plt.tight_layout()
plt.show()