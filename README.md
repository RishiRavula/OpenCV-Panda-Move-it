# Robotics Final Project
Sorting cubes by color using Panda arm

## Members
Kristie Chen, Rishi Ravula, Nolan Prince, Jessica Anz, Alejandro Alvarez

***

## Setup
`bin`, `cube`, `camera` Are all necessary folders that need to go in this path
```
~/ws_moveit/src/panda-gazebo/panda_gazebo/resources/models/
```


`final.py` is the file with the python code.

`panda_gazebo.xacro` is the newly updated panda gazebo file that has friction in the gripper
```
overwrite the current file in the following directory with the file from gitlab
/home/netID/ws_moveit/src/panda_gazebo/franka_ros/franka_description/robots
```

delete the current panda_gazebo.xacro in there and overwrite with this file from gitlab

## Opening Robot and Running Code
Run the following sequence to open MoveIt and Gazebo:
```
roslaunch panda_gazebo start_reach_world.launch
roslaunch panda_gazebo put_robot_in_world.launch
```
Run `python3 final.py` in the terminal to run the python script.
