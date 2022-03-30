# Wind-Turbine-Inspection

This repository contains the simulation files for the autonomous wind turbine inspection project. For this purpose, a time optimal path planner and a Visual tracking MPC is developed. Link to paper:

Installation instructions:
 Install [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)  and [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 


Clone directory 
```bash
cd
```

```bash
git clone https://github.com/abdelhakim96/Wind-Turbine-Inspection
```


Clone the catkin_ws

```bash
cd catkin_ws
```

```bash
cd WTI_catkin
```

Build workspace
```bash
cd WTI_catkin
cd catkin_make
```



Clone the directory:
clone_dir=~/Wind-Turbine-Inspection/WTI_px4_modified     (use the one from the drive)


Build the repo:
Catkin_make


Dependecies:
sudo apt-get ros-mavros-mav-msgs 
setup px4
install_dependencies_and_setup_px4_modified.sh


Add alias:
```bash
alias arm='rosrun mavros mavsafety arm'
alias disarm='rosrun mavros mavsafety disarm'
alias offboard='rosrun mavros mavsys mode -c OFFBOARD'
```


Running the simulation:
```bash
cd Wind-Turbine-Inspection/WTI_px4_modified/shell_scripts/
./run_sitl_gazebo_withWrapper_terminator.sh matrice_100
```
