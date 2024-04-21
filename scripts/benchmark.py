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

def eval_setting(robot_name, t, tight, random_shortcut, planner_name, planning_time_limit, ns):
    base_directory = f'/root/catkin_ws/src/mr_planner/outputs/'
    if planner_name == "AITstar":
        directory = base_directory + f't={planning_time_limit}_AITstar'
    else:
        directory = base_directory + f't={t}_{("tight" if tight else "loose")}'

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
        'output_file': f'{directory}/{robot_name}_benchmark.csv'
        # Add more parameters as needed
    }

    # Run roslaunch
    run_roslaunch('mr_planner', f'{robot_name}.launch', params)

    # Wait for a while to make sure everything has started up
    time.sleep(2)

    script_params = {
        'input': f'{directory}/{robot_name}_benchmark.csv',
        'output': f'{directory}/{robot_name}_benchmark.avg.csv',
    }
    # Run another script
    run_script('count_stats.py', script_params)


# run the evaluations in parallel
processes = []
    
id = 0
envs = ["dual_gp4", "panda_two_rod", "panda_three", "panda_two", 'panda_four']
random_shortcut = True
planner_name = 'AITstar'
shortcut_time = 0
for env in envs:
    for planning_time in [15, 100]:
        for valid in [True, False]:
            ns = f'run_{id}'
            id += 1
            p = mp.Process(target=eval_setting, 
                           args=(env, shortcut_time, valid, random_shortcut, planner_name, planning_time, ns))
            p.start()
            processes.append(p)
            time.sleep(1)

for env in envs:
    ns = f'run_{id}'
    id += 1
    p = mp.Process(target=eval_setting, args=(env, 0, False, ns))
    p.start()
    processes.append(p)
    time.sleep(1)

for p in processes:
    p.join()