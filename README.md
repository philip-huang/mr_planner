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
roslaunch mr_planner planner.launch
```
to launch the lego planner

You can modify the launch file to change the assembly sequence csv file under the ``fullorder_targets_filename`` parameter. You may need to specify the path to configuration files in ``root_pwd`` or ``config_fname``.

## Benchmark planner on different environments

I have included several launch files for running planner in different environments, which are panda_two, panda_three, panda_four, panda_two_rod, and panda_four_bin. To benchmark the performance of the planner / shortcutting algorithm, you can run
```
roslaunch mr_planner panda_two.launch benchmark:=true
```