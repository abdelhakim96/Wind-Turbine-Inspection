#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>


ros::Publisher local_pos_pub;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;



mavros_msgs::State current_state;
mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g; 


struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};



void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_g = *msg;
}




int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");
	// wait for FCU connection
	while (ros::ok() && !current_state_g.connected)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	if(current_state_g.connected)
	{
		ROS_INFO("Connected to FCU");	
		return 0;
	}else{
		ROS_INFO("Error connecting to drone");
		return -1;	
	}
	
	
}




int wait4start()
{
	ROS_INFO("Waiting for user to set mode to OFFBOARD");
	while(ros::ok() && current_state_g.mode != "OFFBOARD")
	{
	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
  	}
  	if(current_state_g.mode == "OFFBOARD")
	{
		ROS_INFO("Mode set to OFFBOARD. Mission starting");
		return 0;
	}else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}




void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_g = *msg;
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  //current_heading_g = psi*(180/M_PI) - local_offset_g;
  //ROS_INFO("Current Heading %f origin", current_heading_g);
  //ROS_INFO("x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}

void set_destination(float x, float y, float z, float psi)
{
	//set_heading(psi);
	//transform map to local
	//float deg2rad = (M_PI/180);
	//float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
	//float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
	//float Zlocal = z;

	//x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
	//y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
	//z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

	waypoint_g.pose.position.x = x;
	waypoint_g.pose.position.y = y;
	waypoint_g.pose.position.z = z;

	local_pos_pub.publish(waypoint_g);
	
}



/**
\ingroup control_functions
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01)
{
	local_pos_pub.publish(waypoint_g);
	
	//check for correct position 
	float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
    float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
    float deltaZ = 0; //abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
    // ROS_INFO("dMag %f", dMag);
    // ROS_INFO("current pose x %F y %f z %f", (current_pose_g.pose.pose.position.x), (current_pose_g.pose.pose.position.y), (current_pose_g.pose.pose.position.z));
    // ROS_INFO("waypoint pose x %F y %f z %f", waypoint_g.pose.position.x, waypoint_g.pose.position.y,waypoint_g.pose.position.z);
    //check orientation
    float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
    float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
    
    float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );



    if( dMag < pos_tolerance && headingErr < heading_tolerance)
	{
		return 1;
	}else{
		return 0;
	}
}









int arm()
{
	//intitialize first waypoint of mission
	set_destination(0,0,0,0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint_g);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
		return 0;
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}
}








int set_speed(float speed__mps)
{
	mavros_msgs::CommandLong speed_cmd;
	speed_cmd.request.command = 178;
	speed_cmd.request.param1 = 1; // ground speed type 
	speed_cmd.request.param2 = speed__mps;
	speed_cmd.request.param3 = -1; // no throttle change
	speed_cmd.request.param4 = 0; // absolute speed
	ROS_INFO("setting speed to %f", speed__mps);
	if(command_client.call(speed_cmd))
	{
		ROS_INFO("change speed command succeeded %d", speed_cmd.response.success);
		return 0;
	}else{
		ROS_ERROR("change speed command failed %d", speed_cmd.response.success);
		ROS_ERROR("change speed result was %d ", speed_cmd.response.result);
		return -1;
	}
	ROS_INFO("change speed result was %d ", speed_cmd.response.result);
	return 0;
}





int init_publisher_subscriber(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
	local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10);
	currentPos = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
	state_sub = controlnode.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
	arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
	land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
	set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
	takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
	command_client = controlnode.serviceClient<mavros_msgs::CommandLong>((ros_namespace + "/mavros/cmd/command").c_str());
	return 0;
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");

	init_publisher_subscriber(gnc_node);

    
  
    //wait4connect();
	//wait4start();
    
    std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 4;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 11500/100;
	nextWayPoint.y = 4850/100;
	nextWayPoint.z = 100/50;
	nextWayPoint.psi = 30;
	waypointList.push_back(nextWayPoint);
	
    

	
    arm();
    mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";	

    mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(20.0);
	int counter = 0;

	ros::Time last_request = ros::Time::now();

	while(ros::ok())
	{   
	    ros::spinOnce();
		rate.sleep();
        if( current_state.mode != "OFFBOARD" &&
				(ros::Time::now() - last_request > ros::Duration(5.0))){
				if( set_mode_client.call(offb_set_mode) &&
						offb_set_mode.response.mode_sent){
						ROS_INFO("Offboard enabled");
				}
				last_request = ros::Time::now();
		} 

	
       

		if(check_waypoint_reached(.3) == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}
		}	
		
	}

	
	return 0;
}
