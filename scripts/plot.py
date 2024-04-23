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
                                    'n_check': int,
                                    'n_valid': int},
                            sep=',')
    
    # Merge the two dataframes on 'start_pose' and 'goal_pose'
    df = pd.merge(df1, df2, on=['start_pose', 'goal_pose'], suffixes=('_df1', '_df2'))
    print(file2, df)

    # Compare the 'flowtime_pre' columns
    df['flowtime_diff'] = (df['flowtime_pre_df2'] - df['flowtime_post_df2']) / df['flowtime_pre_df2'] * 100
    df['makespan_diff'] = (df['makespan_pre_df2'] - df['makespan_post_df2']) / df['makespan_pre_df2'] * 100
    df['t_check_avg_df2'] = df['t_check_df2'] / df['n_check_df2']
    df['valid_perc'] = df['n_valid'] / df['n_check_df2'] * 100
    df['flowtime_diff_per_step'] = (df['flowtime_pre_df2'] - df['flowtime_post_df2']) / df['n_valid']
    
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
                                    'n_check': int,
                                    'n_valid': int},
                            sep=',')
    
    # Merge the two dataframes on 'start_pose' and 'goal_pose'
    df = pd.merge(df1, df2, on=['start_pose', 'goal_pose'], suffixes=('_df1', '_df2'))

    # Compare the 'flowtime_pre' columns
    df['flowtime_diff'] = (df['flowtime_post_df1'] - df['flowtime_post_df2']) / df['flowtime_post_df1'] * 100
    df['makespan_diff'] = (df['makespan_post_df1'] - df['makespan_post_df2']) / df['makespan_post_df1'] * 100
    
    return df


# Plot the data
env = 'panda_two'
t_values = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0]
entries = [('g', 'loose', 'random'), ('b', 'loose', 'iter')]


plt.figure(figsize=(8, 10))
plt.title(f'{env}')
for color, tight, algo in entries:
    flowtime = []
    makespan = []
    n_check = []
    n_valid = []
    flowtime_imp_per_step = []
    for t in t_values:
        # Read the data
        base_dir = '../outputs'
        dir1 = 't=5.0_RRTConnect'
        dir2 = f't={t}_{algo}_{tight}'
        df = read_csv(base_dir, dir1, dir2, env)
        
        flowtime.append((df['flowtime_diff'].mean(), df['flowtime_diff'].std()))
        makespan.append((df['makespan_diff'].mean(), df['makespan_diff'].std()))
        n_check.append((df['n_check_df2'].mean(), df['n_check_df2'].std()))
        n_valid.append((df['n_valid'].mean(), df['n_valid'].std()))
        flowtime_imp_per_step.append((df['flowtime_diff_per_step'].mean(), df['flowtime_diff_per_step'].std()))

    # Plot the data with the specified color and label
    plt.subplot(4, 1, 1)
    
    vals, stds = zip(*flowtime)
    plt.scatter(t_values, vals, label=f'{algo}_{tight}', marker='x', color=color)
    plt.fill_between(t_values, np.array(vals) - np.array(stds), np.array(vals) + np.array(stds), alpha=0.2, color=color)
    
    plt.subplot(4, 1, 2)
    # split array of tuple into two arrays
    vals, stds = zip(*n_check)
    plt.scatter(t_values, vals, label=f'{algo}_{tight}', marker='x', color=color)

    plt.subplot(4, 1, 3)
    vals, stds = zip(*n_valid)
    plt.scatter(t_values, vals, label=f'{algo}_{tight}', marker='x', color=color)

    plt.subplot(4, 1, 4)
    vals, stds = zip(*flowtime_imp_per_step)
    plt.scatter(t_values, vals, label=f'{algo}_{tight}', marker='x', color=color)

    

plt.subplot(4, 1, 1)
df = read_motionplan_csv('../outputs/', 't=5.0_RRTConnect', 't=100.0_BITstar', env)
ait_perc_improv = df['flowtime_diff'].mean()
plt.axhline(y=ait_perc_improv, color='b', linestyle='--', label='BIT*')

# log axis for x
plt.xscale('log')
# lower bound the y axis to 0
#plt.ylim(bottom=0)
plt.ylabel('Perct Improvement (%)')
plt.legend()


plt.subplot(4, 1, 2)
plt.xscale('log')
plt.yscale('log')
plt.ylabel('# Shortcut Checked')

plt.subplot(4, 1, 3)
plt.xscale('log')
plt.ylabel('# Shortcut Valid')

plt.subplot(4, 1, 4)
plt.xscale('log')
plt.xlabel('Time (s)')
plt.ylabel('flow_improv_per_step')

plt.tight_layout()
plt.savefig(f'../outputs/plots/{env}.png')
plt.show()