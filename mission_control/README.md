# mavros_controllers
[![Build Test](https://github.com/Jaeyoung-Lim/mavros_controllers/workflows/Build%20Test/badge.svg)](https://github.com/Jaeyoung-Lim/mavros_controllers/actions?query=workflow%3A%22Build+Test%22) [![DOI](https://zenodo.org/badge/140596755.svg)](https://zenodo.org/badge/latestdoi/140596755)

Controllers for controlling MAVs using the [mavros](https://github.com/mavlink/mavros) package in OFFBOARD mode.

## Getting Started
### Install PX4 SITL(Only to Simulate)
Follow the instructions as shown in the [ROS with AirSim Simulation PX4 Documentation](https://microsoft.github.io/AirSim/px4_sitl/)
To check if the necessary environment is setup correctly, you can run the gazebo SITL using the following command

```bash
cd <Firmware_directory>
make px4_sitl_default none_iris
```
You can run the rest of the roslaunch files in the same terminal

```bash
 roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

You will need to source the PX4 environment in every new terminal you open to launch mavros_controllers. 

### Installing mavros_controllers

Create a catkin workspace:

This folder will probably be already created since the previous process would have created it. If it is not present, do:

```bash
# catkin_ws = "Your ws name" 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --merge-devel
cd src
wstool init
```

###### Clone this repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/Jaeyoung-Lim/mavros_controllers # TODO Change to ours
```

Now continue either with wstool to automatically download dependencies or download them manually.

###### With wstool

wstool automates the installation of dependencies and updates all packages. If you have no problem updating the packages required by mavros_controllers and/or any other packages, follow this procedure. If not, follow the next 'Manually Download dependencies and build' section.

```bash
cd ~/catkin_ws
wstool merge -t src src/mavros_controller/dependencies.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO
catkin build
source ~/catkin_ws/devel/setup.bash
```


###### Manually Download dependencies and build

If you did not install with wstool, you need to manually download the dependencies:
- [catkin_simple](https://github.com/catkin/catkin_simple) # Needed in Eigen catkin
- [eigen_catkin](https://github.com/ethz-asl/eigen_catkin) 
- [mav_comm](https://github.com/ethz-asl/mav_comm)

Do:

```bash
cd ~/catkin_ws/src
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/ethz-asl/eigen_catkin
git clone https://github.com/ethz-asl/mav_comm
git clone https://github.com/ethz-asl/json_catkin
```

Build all the packages:

```bash
cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
```

## Running the code
The following launch file enables the geometric controller to follow a circular trajectory

``` bash
roslaunch offboard_control offboard_control.launch
```

## Nodes
`mavros_controllers` include the following packages.

### offboard_control_node

The geometric controller publishes and subscribes the following topics.
- Parameters
    - /geometric_controller/mavname (default: "iris")
    - /geometric_controller/ctrl_mode (default: MODE_BODYRATE)
    - /geometric_controller/enable_sim (default: true)
    - /geometric_controller/enable_gazebo_state (default: false)
    - /geometric_controller/max_acc (default: 7.0)
    - /geometric_controller/yaw_heading (default: 0.0)
    - /geometric_controller/drag_dx (default: 0.0)
    - /geometric_controller/drag_dy (default: 0.0)
    - /geometric_controller/drag_dz (default: 0.0)
    - /geometric_controller/attctrl_constant (default: 0.2)
    - /geometric_controller/normalizedthrust_constant (default: 0.1)

- Published Topics
	- command/bodyrate_command ( [mavros_msgs/AttitudeTarget](http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html) )
	- reference/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )

- Subscribed Topics
	- reference/setpoint ( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )
	- /mavros/state ( [mavr0s_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html) )
	- /mavros/local_position/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )
	- /gazebo/model_states( [gazebo_msgs/ModelStates](http://docs.ros.org/kinetic/api/gazebo_msgs/html/msg/ModelState.html) )
	- /mavros/local_position/velocity( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )


### Build issues:


###### catkin_simple() or eigen_catkin() not found

 This should not have happened if you clone the catkin_simple and eigen_catkin repositories. Try again:

```bash
cd ~/catkin_ws/src
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/ethz-asl/eigen_catkin
cd ~/catkin_ws
catkin build mavros_controllers
source ~/catkin_ws/devel/setup.bash
```

- Refer to [this issue](https://github.com/Jaeyoung-Lim/mavros_controllers/issues/61).
