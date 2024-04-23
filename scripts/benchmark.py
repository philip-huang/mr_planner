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

def eval_setting(ns, robot_name, load_tpg, t, tight, random_shortcut, planner_name, planning_time_limit,
                 forward_doubleloop, backward_doubleloop, forward_singleloop):
    base_directory = f'/root/catkin_ws/src/mr_planner/outputs/'
    if load_tpg:
        directory = base_directory + f't={t}_{"random" if random_shortcut else "iter"}_{("tight" if tight else "loose")}'
    else:
        directory = base_directory + f't={planning_time_limit}_{planner_name}'
    tpg_directory = base_directory + f'tpgs/t={planning_time_limit}_{planner_name}_{robot_name}'

    if not os.path.exists(directory):
        # If not, create the directory
        os.makedirs(directory)

    # Set parameters
    params = {
        'benchmark': 'true',
        'use_rviz': 'false',
        'random_shortcut': 'true' if random_shortcut else 'false',
        'shortcut_time': str(t),
        'tight_shortcut': 'true' if tight else 'false',
        'planner_name': planner_name,
        'planning_time_limit': planning_time_limit,
        'ns': ns,
        'output_file': f'{directory}/{robot_name}_benchmark.csv', 
        'load_tpg': 'true' if load_tpg else 'false',
        'tpg_savedir': tpg_directory,
        'forward_doubleloop': 'true' if forward_doubleloop else 'false',
        'backward_doubleloop': 'true' if backward_doubleloop else 'false',
        'forward_singleloop': 'true' if forward_singleloop else 'false',
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
def add_planner_processes(envs, id = 0):
    processes = []

    random_shortcut = True
    shortcut_time = 0.0
    load_tpg = False
    tight = False
    for env in envs:
        for planner_name, planning_time in [("BITstar", 100.0)]:
            ns = f'run_{id}'
            id += 1
            p = mp.Process(target=eval_setting, 
                            args=(ns, env, load_tpg, shortcut_time, tight, random_shortcut, 
                                  planner_name, planning_time, False, False, True))
            p.start()
            processes.append(p)
            time.sleep(1)
         
    return processes, id

def add_tpg_processes(envs, shortcut_ts, id = 0):
    processes = []

    load_tpg = True
    random_shortcut = True
    planning_time = 5.0
    planner_name = 'RRTConnect'
    for env in envs:
        for shortcut_t in shortcut_ts:
            for tight in [True, False]:
                ns = f'run_{id}'
                id += 1
                p = mp.Process(target=eval_setting, 
                                args=(ns, env, load_tpg, shortcut_t, tight, random_shortcut, 
                                      planner_name, planning_time, False, False, True))
                p.start()
                processes.append(p)
                time.sleep(1)

def add_baseline_processes(envs, shortcut_ts, loop_type, id = 0):
    processes = []

    # add baselines
    load_tpg = True
    random_shortcut = False
    forward_doubleloop = True if loop_type == 'forward_doubleloop' else False
    backward_doubleloop = True if loop_type == 'backward_doubleloop' else False
    forward_singleloop = True if loop_type == 'forward_singleloop' else False

    tight = False
    planning_time = 5.0
    planner_name = 'RRTConnect'

    for env in envs:
        for shortcut_t in shortcut_ts:
            ns = f'run_{id}'
            id += 1
            p = mp.Process(target=eval_setting, 
                            args=(ns, env, load_tpg, shortcut_t, tight, random_shortcut, planner_name, planning_time,
                                forward_doubleloop, backward_doubleloop, forward_singleloop))
            p.start()
            processes.append(p)
            time.sleep(1)
    return processes, id


if __name__ == "__main__":
    envs = ["panda_three", "panda_four", "panda_two_rod"]

    # processes, id = add_planner_processes(envs)
    # for p in processes:
    #     p.join()

    shortcut_ts = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0]
    # processes, id = add_tpg_processes(envs, shortcut_ts)
    # for p in processes:
    #     p.join()

    processes, id = add_baseline_processes(envs, shortcut_ts, 'forward_singleloop')
    for p in processes:
        p.join()