## Multirobot Planner 

This pacakage is a moveit-based multi-robot planning for lego manipulation

# Installation
Build the docker image and run it inside docker
```
cd docker && bash build.sh
```

Or if you are not using the docker file, make sure to download moveit, the [lego Manipulation](https://github.com/intelligent-control-lab/Robotic_Lego_Manipulation) and [digital twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin.git). 

## Run demo for lego building

Lunch the docker or a terminal, then run
```
roslaunch mr_planner lego.launch
```
to launch the lego planner

You can modify the launch file to change the assembly sequence csv file under the ``fullorder_targets_filename`` parameter. You may need to specify the path to configuration files in ``root_pwd`` or ``config_fname``.

## Benchmark planner on different environments

I have included several launch files for running planner in different environments, which are panda_two, panda_three, panda_four, panda_two_rod, and panda_four_bin. To benchmark the performance of the planner / shortcutting algorithm, you can run
```
roslaunch mr_planner panda_two.launch benchmark:=true
```

## Moveit
Currently the backend is implemented by moveit. Their main purpose is for visualizing, collision checking, and executing the planned trajectory. We also sometimes use the default planner (RRTConnect, BITStar) in moveit for planning in joint state space.

We have five environments (dual_gp4, panda_two, panda_three, panda_four, panda_four_bins, panda_two_rod) to test 

In each environment we define a joint planning group for joint planning and execution, as well as individual planning groups for each robot.

The soruce code for these environemnts are in the [moveit_config](https://github.com/philip-huang/moveit_configs) repo.

To run moveit's default planner, you can use one of the default demo.launch files, e.g.

```
roslaunch panda_two_rod_moveit_config demo.launch
```

## Code Structure

- `include`: API of the library
    - `instance.h`: Class for the planning scene
    - `logger.h`: Utilities for logging
    - `planner.h`: Implements a multi-robot planning interface
    - `SingleAgentPlanner.h`: Implements the single agent planning algorithm
    - `task.h`: Defines the activity graph
    - `tpg.h`: Implements the Temporal Plan Graph execution policy and post-processing algorithm
    - `adg.h`: Extends the TPG for multi-task activity graph

- `src`: Code for the library and executable
    - `demo_node.cpp`: Executable for testing single-step planning
    - `lego_node.cpp`: Executable for sequential planning for lego assembly

- `launch`: 
    - `lego.launch`: Launch file for testing the lego assembly
    - `dual_gp4.launch`, `panda_two.launch`, `panda_two_rod.launch`, `panda_three.launch`, `panda_four.launch`, `panda_four_bins.launch`: Launch files for testing the single agent planning

- `scripts`:
    - `benchmark.py`: Python scripts for benchmarking motion planning/TPG processing in parallel
    - `plot.py`: Visualize the results 

- `assembly_steps`: Stores the sequential task plan for assembling different lego structures