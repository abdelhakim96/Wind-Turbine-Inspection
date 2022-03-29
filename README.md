# Wind-Turbine-Inspection
This repo contains the code for the autonomous wind turbine inspection project. AirSim is used in this project.





clone_dir=~/Wind-Turbine-Inspection/WTI_px4_modified



export GAZEBO_MODEL_PATH=:$clone_dir/Tools/sitl_gazebo/models
export GAZEBO_PLUGIN_PATH=:$clone_dir/Tools/sitl_gazebo/Build/devel/lib
export GAZEBO_MODEL_DATABASE_URI=:http://get.gazebosim.org
export SITL_GAZEBO_PATH=:$clone_dir/Tools/sitl_gazebo


alias arm='rosrun mavros mavsafety arm'
alias disarm='rosrun mavros mavsafety disarm'
alias offboard='rosrun mavros mavsys mode -c OFFBOARD'



cd Wind-Turbine-Inspection-main/WTI_px4_modified/shell_scripts/


./run_sitl_gazebo_withWrapper_terminator.sh matrice 100



sudo apt-get ros-mavros-mav-msgs 
use the px4_modified from drive
install_dependencies_and_setup_px4_modified.sh
