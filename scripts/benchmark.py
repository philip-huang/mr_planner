import subprocess
import time
import os
import multiprocessing as mp

def run_roslaunch(package, launch_file, params):
    # Start roslaunch
    roslaunch = subprocess.Popen(['roslaunch', package, launch_file, *['{}:={}'.format(k, v) for k, v in params.items()]])

    # Wait for roslaunch to finish
    roslaunch.wait()

def run_script(script_path, params):
    # Run another script
    script = subprocess.Popen(['python3', script_path, *['--{}={}'.format(k, v) for k, v in params.items()]])
    script.wait()

def eval_setting(robot_name, load_tpg, t, tight, random_shortcut, planner_name, planning_time_limit, ns):
    base_directory = f'/root/catkin_ws/src/mr_planner/outputs/'
    if planner_name == "AITstar":
        directory = base_directory + f't={planning_time_limit}_AITstar'
    else:
        directory = base_directory + f't={t}_{("tight" if tight else "loose")}'
    tpg_directory = base_directory + f'tpgs/t={planning_time_limit}_{planner_name}_{robot_name}'

    if not os.path.exists(directory):
        # If not, create the directory
        os.makedirs(directory)

    # Set parameters
    params = {
        'benchmark': 'true',
        'use_rviz': 'false',
        'random_shortcut': 'true' if random_shortcut else 'false',
        'random_shortcut_time': str(t),
        'tight_shortcut': 'true' if tight else 'false',
        'planner_name': planner_name,
        'planning_time_limit': planning_time_limit,
        'ns': ns,
        'output_file': f'{directory}/{robot_name}_benchmark.csv', 
        'load_tpg': 'true' if load_tpg else 'false',
        'tpg_savedir': tpg_directory,
        # Add more parameters as needed
    }

    # Run roslaunch
    run_roslaunch('mr_planner', f'{robot_name}.launch', params)

    # Wait for a while to make sure everything has started up
    time.sleep(2)

    if load_tpg:
        script_params = {
            'input': f'{directory}/{robot_name}_benchmark.csv',
            'output': f'{directory}/{robot_name}_benchmark.avg.csv',
        }
        # Run another script
        run_script('count_stats.py', script_params)


# run the evaluations in parallel
processes = []
    
id = 0
envs = ["dual_gp4", 'panda_two', 'panda_three', 'panda_four', 'panda_two_rod']
random_shortcut = True
planner_name = 'RRTConnect'
shortcut_time = 0.01
load_tpg = False
planning_time = 5.0
valid = False
for env in envs:
    ns = f'run_{id}'
    id += 1
    p = mp.Process(target=eval_setting, 
                    args=(env, load_tpg, shortcut_time, valid, random_shortcut, planner_name, planning_time, ns))
    p.start()
    processes.append(p)
    time.sleep(1)

# Wait for all processes to finish

for p in processes:
    p.join()

load_tpg = True

for env in envs:
    for shortcut_t in [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]:
        for valid in [True, False]:
            ns = f'run_{id}'
            id += 1
            p = mp.Process(target=eval_setting, 
                            args=(env, load_tpg, shortcut_t, valid, random_shortcut, planner_name, planning_time, ns))
            p.start()
            processes.append(p)
            time.sleep(1)

for p in processes:
    p.join()