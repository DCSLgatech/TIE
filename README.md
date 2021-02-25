# TIE
Time Informed Exploration code

Repository for Time-Informed Exploration, which uses ideas from reachability to focus search for sampling-based kino-dynamic planning.

## Dependencies 
This repository is a ROS-Kinetic package. All the code is written using the popular OMPL framework. 
Also requires python and matplotlib.

Clone this package into your catkin workspace and compile with 
```
cd catkin_ws/src
cd ..
catkin_make
```
For running a 2D planning case, navigate to tie/src and do
```
rosrun tie toy2d_propogate_test
python plothpath.py
```
to quickly plot the graph and solution.
