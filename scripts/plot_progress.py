import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from count_stats import average_improvement

# Read the data from the csv file
def read_csv(base_dir, seed, algo):
    file1 = f'{base_dir}/progress_{seed}_{algo}.csv'
    df = pd.read_csv(file1, dtype={'start_pose' : str,
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

    return df

# Plot the data
seeds = [1, 2, 3, 4]

entries = [#('b', 'o', 'iter_loose', 'Iterative'),
            #('r', 's', 'bwd_diter_loose', 'Backward Double'),
            ('g', 'x', 'loose_', 'Random'),
            ('purple', 'D', 'tight_', 'Random + Tight'),
        ]
metric = 'makespan'

plt.figure(figsize=(8, 12))
plt.rcParams.update({'font.size': 15})

for color, marker, algo, label in entries:
    base_dir = '/home/philip/catkin_ws/src/mr_planner/outputs/lego'

    data_frames = [read_csv(base_dir, seed, algo) for seed in seeds]
    all_data = pd.concat(data_frames)
    grouped_data = all_data.groupby('n_valid').agg({
        't_check': ['mean'],
        'makespan_post': ['mean', 'std'],
        'n_check': ['mean', 'std'],
        'n_valid': ['mean', 'std'],
    }).reset_index()
    grouped_data.columns = ['n_valid', 't_check_mean', 'makespan_mean', 'makespan_std', 'n_check_mean', 'n_check_std', 'n_valid_mean', 'n_valid_std']


    # Convert to NumPy arrays for plotting
    t_check = grouped_data['t_check_mean'].to_numpy()
    makespan_mean = grouped_data['makespan_mean'].to_numpy()
    makespan_std = grouped_data['makespan_std'].to_numpy()
    n_check_mean = grouped_data['n_check_mean'].to_numpy()
    n_check_std = grouped_data['n_check_std'].to_numpy()
    n_valid_mean = grouped_data['n_valid_mean'].to_numpy()
    n_valid_std = grouped_data['n_valid_std'].to_numpy()
    print(makespan_std)


    # Plot makespan
    plt.subplot(3, 1, 1)
    plt.plot(t_check, makespan_mean, color=color, label=label, marker=marker)
    plt.fill_between(t_check, makespan_mean - makespan_std, makespan_mean + makespan_std, color=color, alpha=0.2)


    # Plot n_check
    plt.subplot(3, 1, 2)
    plt.plot(t_check, n_check_mean, color=color, label=label, marker=marker)
    plt.fill_between(t_check, n_check_mean - n_check_std, n_check_mean + n_check_std, color=color, alpha=0.2)


    # Plot n_valid
    plt.subplot(3, 1, 3)
    plt.plot(t_check, n_valid_mean, color=color, label=label, marker=marker)
    plt.fill_between(t_check, n_valid_mean - n_valid_std, n_valid_mean + n_valid_std, color=color, alpha=0.2)

# log axis for x
plt.subplot(3, 1, 1)
# lower bound the y axis to 0
#plt.ylim(bottom=0)
plt.ylabel('Makespan')

plt.subplot(3, 1, 2)
plt.ylabel('# Shortcut Checked')
plt.legend()

plt.subplot(3, 1, 3)
plt.ylabel('# Shortcut Valid')

plt.tight_layout()
plt.savefig(f'../outputs/plots/lego.png')
plt.show()