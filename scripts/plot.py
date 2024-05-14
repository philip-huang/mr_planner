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
                                    'n_valid_df2': int},
                            sep=',')
    
    df2.rename(columns={'n_valid': 'n_valid_df2'}, inplace=True)
    # Merge the two dataframes on 'start_pose' and 'goal_pose'
    df = pd.merge(df1, df2, on=['start_pose', 'goal_pose'], suffixes=('_df1', '_df2'))

    # Compare the 'flowtime_pre' columns
    df['flowtime_diff'] = (df['flowtime_pre_df2'] - df['flowtime_post_df2']) / df['flowtime_pre_df2'] * 100
    df['makespan_diff'] = (df['makespan_pre_df2'] - df['makespan_post_df2']) / df['makespan_pre_df2'] * 100
    df['t_check_avg_df2'] = df['t_check_df2'] / df['n_check_df2']
    df['valid_perc'] = df['n_valid_df2'] / df['n_check_df2'] * 100
    df['flowtime_diff_per_step'] = (df['flowtime_pre_df2'] - df['flowtime_post_df2']) / df['n_valid_df2']
    df['makespan_diff_per_step'] = (df['makespan_pre_df2'] - df['makespan_post_df2']) / df['n_valid_df2']
    
    df['flowtime_diff_per_step'] = df['flowtime_diff_per_step'].replace(-np.inf, np.nan).clip(lower=0)
    df['makespan_diff_per_step'] = df['makespan_diff_per_step'].replace(-np.inf, np.nan).clip(lower=0)

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
                                    'n_valid_df2': int},
                            sep=',')
    
    # Merge the two dataframes on 'start_pose' and 'goal_pose'
    df = pd.merge(df1, df2, on=['start_pose', 'goal_pose'], suffixes=('_df1', '_df2'))

    # Compare the 'flowtime_pre' columns
    df['flowtime_diff'] = (df['flowtime_post_df1'] - df['flowtime_post_df2']) / df['flowtime_post_df1'] * 100
    df['makespan_diff'] = (df['makespan_post_df1'] - df['makespan_post_df2']) / df['makespan_post_df1'] * 100
    
    return df


# Plot the data
env = 'dual_gp4'
t_values = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0]
#t_values = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0]
#t_values = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0]
entries = [#('b', 'o', 'iter_loose', 'Iterative'),
            #('r', 's', 'bwd_diter_loose', 'Backward Double'),
            #('g', 'x', 'random_loose', 'Random'),
            ('purple', 'D', 'random_tight', 'Random + Tight'),
            #('orange', 'P', 'random_loose_biased', 'Random + Biased'),
            ('cyan', 'v', 'random_tight_biased', 'Random + Tight + Biased'),
        ]
metric = 'makespan'

plt.figure(figsize=(8, 12))
plt.title(f'{env}')
plt.rcParams.update({'font.size': 15})

for color, marker, algo, label in entries:
    flowtime = []
    makespan = []
    n_check = []
    n_valid = []
    flowtime_imp_per_step = []
    makespan_imp_per_step = []
    t_val_actual = []
    for t in t_values:
        # Read the data
        base_dir = '../outputs'
        dir1 = 't=5.0_RRTConnect'
        dir2 = f't={t}_{algo}'
        df = read_csv(base_dir, dir1, dir2, env)
        
        flowtime.append((df[f'flowtime_diff'].mean(), df['flowtime_diff'].std()))
        makespan.append((df['makespan_diff'].mean(), df['makespan_diff'].std()))
        n_check.append((df['n_check_df2'].mean(), df['n_check_df2'].std()))
        n_valid.append((df['n_valid_df2'].mean(), df['n_valid_df2'].std()))
        t_val_actual.append(df['t_shortcut_df2'].mean())
        flowtime_imp_per_step.append((df['flowtime_diff_per_step'].dropna().mean(), df['flowtime_diff_per_step'].dropna().std()))
        makespan_imp_per_step.append((df['makespan_diff_per_step'].dropna().mean(), df['makespan_diff_per_step'].dropna().std()))

    # Plot the data with the specified color and label
    plt.subplot(4, 1, 1)
    
    vals, stds = zip(*makespan) if metric == 'makespan' else zip(*flowtime)
    plt.plot(t_val_actual, vals, label=label, marker=marker, color=color)
    plt.fill_between(t_val_actual, np.array(vals) - np.array(stds), np.array(vals) + np.array(stds), alpha=0.2, color=color)
    
    plt.subplot(4, 1, 2)
    # split array of tuple into two arrays
    vals, stds = zip(*n_check)
    plt.plot(t_val_actual, vals, label=label, marker=marker, color=color)

    plt.subplot(4, 1, 3)
    vals, stds = zip(*n_valid)
    plt.plot(t_val_actual, vals, label=label, marker=marker, color=color)

    plt.subplot(4, 1, 4)
    vals, stds = zip(*makespan_imp_per_step) if metric == 'makespan' else zip(*flowtime_imp_per_step)
    plt.plot(t_val_actual, vals, label=label, marker=marker, color=color)

    

plt.subplot(4, 1, 1)
df = read_motionplan_csv('../outputs/', 't=5.0_RRTConnect', 't=100.0_BITstar', env)
ait_perc_improv = df['flowtime_diff'].mean() if metric == 'flowtime' else df['makespan_diff'].mean()
plt.axhline(y=ait_perc_improv, color='k', linestyle='--', label='BIT*')

# log axis for x
plt.xscale('log')
# lower bound the y axis to 0
#plt.ylim(bottom=0)
plt.ylabel('Improvement (%)')

plt.subplot(4, 1, 2)
plt.xscale('log')
plt.yscale('log')
plt.ylabel('# Shortcut Checked')
plt.legend()

plt.subplot(4, 1, 3)
plt.xscale('log')
plt.ylabel('# Shortcut Valid')

plt.subplot(4, 1, 4)
plt.xscale('log')
plt.xlabel('Time (s)')
plt.ylabel('Reduction (s) per Step')

plt.tight_layout()
plt.savefig(f'../outputs/plots/{env}.png')
plt.show()