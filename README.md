# Wind-Turbine-Inspection

This repository contains the simulation files for the autonomous wind turbine inspection project. For this purpose, a time optimal path planner and a Visual tracking MPC is developed. Link to paper:

Installation instructions:
- Install Ubuntu 18.04 and ROS melodic

Clone the directory:
clone_dir=~/Wind-Turbine-Inspection/WTI_px4_modified     (use the one from the drive)


Build the repo:
Catkin_make


Dependecies:
sudo apt-get ros-mavros-mav-msgs 
setup px4
install_dependencies_and_setup_px4_modified.sh


Add alias:
alias arm='rosrun mavros mavsafety arm'
alias disarm='rosrun mavros mavsafety disarm'
alias offboard='rosrun mavros mavsys mode -c OFFBOARD'



Running the simulation:
cd Wind-Turbine-Inspection/WTI_px4_modified/shell_scripts/
./run_sitl_gazebo_withWrapper_terminator.sh matrice_100
