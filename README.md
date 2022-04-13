# Autonomous Wind Turbine Inspection Project

This repository contains the simulation files for the autonomous wind turbine inspection project. For this purpose, a time optimal path planner and a Visual tracking MPC is developed. Link to paper:

Installation instructions:
 Install [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)  and [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 


Clone directory 
```bash
cd
git clone https://github.com/abdelhakim96/Wind-Turbine-Inspection
```


Clone the catkin_ws

```bash
cd catkin_ws
cd WTI_catkin
```

Build workspace
```bash
cd WTI_catkin
cd catkin_make
```

Download PX4 folder 


Dependecies and setup px4:
```bash
sudo apt-get ros-mavros-mav-msgs 
cd Wind-Turbine-Inspection/WTI_px4_modified
install_dependencies_and_setup_px4_modified.sh
```

Add alias in bash rc:
```bash
alias arm='rosrun mavros mavsafety arm'
alias disarm='rosrun mavros mavsafety disarm'
alias offboard='rosrun mavros mavsys mode -c OFFBOARD'
```


Starting the simulation:
```bash
cd Wind-Turbine-Inspection/WTI_px4_modified/shell_scripts/
./run_sitl_gazebo_withWrapper_terminator.sh matrice_100
```

Running the Inspection Planner

launch rqt_reconfigure trajectory 

```bash
roslaunch dji_m100_trajectory m100_trajectory_v2_indoor.launch 
```

Activate traj_on 


in the  terminal arm the dorne and set the mode to offboard on by typing the following commands
```bash
arm
offboard
```


The drone will take off.


Next launch the VT-MPC.

```bash
roslaunch quaternion_point_traj_nmpc quaternion_point_traj_nmpc.launch
```




