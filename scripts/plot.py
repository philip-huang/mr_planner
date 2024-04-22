import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from count_stats import average_improvement

# Read the data from the csv file
def read_csv(base_dir, dir1, dir2, env):
    file1 = f'{base_dir}/{dir1}/{env}_benchmark.csv'
    df1 = pd.read_csv(file1, dtype={'start_pose' : str,
                                    'goal_pose': str,
                                   'flowtime_pre': float,
                                   'makespan_pre': float,
                                    'flowtime_post': float,
                                    'makespan_post': float,
                                    't_init': float,
                                    't_shortcut': float,
                                    't_mcp': float,
                                    't_check': float,
                                    'n_check': int},
                            sep=',')
    file2 = f'{base_dir}/{dir2}/{env}_benchmark.csv'
    df2 = pd.read_csv(file2, dtype={'start_pose' : str,
                                    'goal_pose': str,
                                   'flowtime_pre': float,
                                   'makespan_pre': float,
                                    'flowtime_post': float,
                                   'makespan_post': float,
                                    't_init': float,
                                    't_shortcut': float,
                                    't_mcp': float,
                                    't_check': float,
                                    'n_check': int},
                            sep=',')
    
    # Merge the two dataframes on 'start_pose' and 'goal_pose'
    df = pd.merge(df1, df2, on=['start_pose', 'goal_pose'], suffixes=('_df1', '_df2'))

    # Compare the 'flowtime_pre' columns
    df['flowtime_diff'] = (df['flowtime_pre_df2'] - df['flowtime_post_df2']) / df['flowtime_pre_df2'] * 100
    print(df['flowtime_diff'])
    df['makespan_diff'] = (df['makespan_pre_df2'] - df['makespan_post_df2']) / df['makespan_pre_df2'] * 100
    
    return df

def read_motionplan_csv(base_dir, dir1, dir2, env):
    file1 = f'{base_dir}/{dir1}/{env}_benchmark.csv'
    df1 = pd.read_csv(file1, dtype={'start_pose' : str,
                                    'goal_pose': str,
                                   'flowtime_pre': float,
                                   'makespan_pre': float,
                                    'flowtime_post': float,
                                    'makespan_post': float,
                                    't_init': float,
                                    't_shortcut': float,
                                    't_mcp': float,
                                    't_check': float,
                                    'n_check': int},
                            sep=',')
    file2 = f'{base_dir}/{dir2}/{env}_benchmark.csv'
    df2 = pd.read_csv(file2, dtype={'start_pose' : str,
                                    'goal_pose': str,
                                   'flowtime_pre': float,
                                   'makespan_pre': float,
                                    'flowtime_post': float,
                                   'makespan_post': float,
                                    't_init': float,
                                    't_shortcut': float,
                                    't_mcp': float,
                                    't_check': float,
                                    'n_check': int},
                            sep=',')
    
    # Merge the two dataframes on 'start_pose' and 'goal_pose'
    df = pd.merge(df1, df2, on=['start_pose', 'goal_pose'], suffixes=('_df1', '_df2'))

    # Compare the 'flowtime_pre' columns
    df['flowtime_diff'] = (df['flowtime_post_df1'] - df['flowtime_post_df2']) / df['flowtime_post_df1'] * 100
    print(df['flowtime_diff'])
    df['makespan_diff'] = (df['makespan_post_df1'] - df['makespan_post_df2']) / df['makespan_post_df1'] * 100
    
    return df


# Plot the data
env = 'dual_gp4'
t_values = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]
tight_values = ['loose']
colors = ['g']

plt.figure()
plt.title(f'{env}')
for color, tight in zip(colors, tight_values):
    vals = []
    stds = []
    for t in t_values:
        # Read the data
        base_dir = '../outputs'
        dir1 = 't=5.0_RRTConnect'
        dir2 = f't={t}_random_{tight}'
        df = read_csv(base_dir, dir1, dir2, env)
        
        vals.append(df['flowtime_diff'].mean())
        stds.append(df['flowtime_diff'].std())

    
    # Plot the data with the specified color and label
    print(vals)
    plt.scatter(t_values, vals, label=f'{tight}', marker='x', color=color)

    # Plot the stddev as shaded region with the same color but without a label
    plt.fill_between(t_values, np.array(vals) - np.array(stds), np.array(vals) + np.array(stds), alpha=0.2, color=color)

df = read_motionplan_csv('../outputs/', 't=5.0_RRTConnect', 't=15.0_BITstar', env)
ait_perc_improv = df['flowtime_diff'].mean()
plt.axhline(y=ait_perc_improv, color='r', linestyle='--', label='AIT* 15sec')

df = read_csv('../outputs/', 't=5.0_RRTConnect', 't=0.0__loose', env)
baseline_perc_improv = df['flowtime_diff'].mean()
baseline_shortcut_t = df['t_shortcut_df2'].mean()
plt.scatter([baseline_shortcut_t], [baseline_perc_improv], label='Baseline', marker='x', color='b')

# log axis for x
plt.xscale('log')
plt.xlabel('Time (s)')
plt.ylabel('Perct Improvement (%)')

plt.legend()
plt.tight_layout()
plt.savefig(f'../outputs/plots/{env}.png')
plt.show()