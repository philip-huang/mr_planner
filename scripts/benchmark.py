import subprocess
import time
import os

def run_roslaunch(package, launch_file, params):
    # Start roslaunch
    roslaunch = subprocess.Popen(['roslaunch', package, launch_file, *['{}:={}'.format(k, v) for k, v in params.items()]])

    # Wait for roslaunch to finish
    roslaunch.wait()

def run_script(script_path, params):
    # Run another script
    script = subprocess.Popen(['python3', script_path, *['--{}={}'.format(k, v) for k, v in params.items()]])
    script.wait()


for env in ["panda_two_rod", "panda_three", "panda_four"]:

    # Set parameters
    params = {
        'benchmark': 'true',
        'random_shortcut_time': '0.1',
        'tight_shortcut': 'true',
        'output_file': f'/root/catkin_ws/src/mr_planner/outputs/{env}_benchmark.csv'
        # Add more parameters as needed
    }

    # Run roslaunch
    run_roslaunch('mr_planner', f'{env}.launch', params)

    # Wait for a while to make sure everything has started up
    time.sleep(5)

    script_params = {
        'input': f'../outputs/{env}_benchmark.csv',
        'output': f'../outputs/{env}_benchmark_avg.csv',
    }
    # Run another script
    run_script('count_stats.py', script_params)