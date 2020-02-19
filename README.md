# DWA local planner2

This local planner complements DWA(Dynamic Window Approach), which is mainly used as a local planner in ROS, to have **avoiding function for dynamic environments**. 

The order of the proposed avoidance algorithm is followed:

1) Predicts trajectory of nearby obstacles by 2D-LiDAR sensor
2) Calculates time-to-collision for each obstacle
3) Calculates collision avoidance probability based on time-to-collision
4) Add collision avoidance probability in the cost function of DWA, find a path with the highest value

This dynamic obstacle avoidance algorithm avoids robot from driving at a speed which is more likely to cause a collision. 

DWA local planner original code for ROS is available [here](https://github.com/ros-planning/navigation/tree/kinetic-devel/dwa_local_planner/).

