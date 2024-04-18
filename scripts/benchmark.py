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

def eval_setting(robot_name, t, tight, ns):
    directory = f'/root/catkin_ws/src/mr_planner/outputs/t={t}_{("tight" if tight else "loose")}'
    if not os.path.exists(directory):
        # If not, create the directory
        os.makedirs(directory)

    # Set parameters
    params = {
        'benchmark': 'true',
        'use_rviz': 'false',
        'random_shortcut_time': str(t),
        'tight_shortcut': 'true' if tight else 'false',
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
for env in ["panda_three", "panda_two"]:
    for t in [0.1, 0.5]:
        for valid in [True]:
            ns = f'run_{id}'
            id += 1
            p = mp.Process(target=eval_setting, args=(env, t, valid, ns))
            p.start()
            processes.append(p)
            time.sleep(1)

for p in processes:
    p.join()