#ifndef _NMPC_PC_MAIN_H
#define _NMPC_PC_MAIN_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
// #include<mavros_msgs/ExtendedState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/Thrust.h>
#include <tf/tf.h>

#include <nmpc_pc_learning.h>

// Subscribers
ros::Subscriber state_sub;

ros::Subscriber ref_trajectory_sub;
ros::Subscriber ref_velocity_sub;
ros::Subscriber ref_yaw_sub;
ros::Subscriber pos_sub;
ros::Subscriber vel_sub;
ros::Subscriber dist_Fx_predInit_sub;
ros::Subscriber dist_Fy_predInit_sub;
ros::Subscriber dist_Fz_predInit_sub;
ros::Subscriber dist_Fx_data_sub;
ros::Subscriber dist_Fy_data_sub;
ros::Subscriber dist_Fz_data_sub;

// Publishers
ros::Publisher att_throttle_pub;
ros::Publisher attitude_pub;
ros::Publisher nmpc_cmd_rpy_pub;
ros::Publisher nmpc_cmd_Fz_pub;
ros::Publisher nmpc_cmd_exeTime_pub;
ros::Publisher nmpc_cmd_kkt_pub;
ros::Publisher nmpc_cmd_obj_pub;

nmpc_struct_ nmpc_struct;

std::string mocap_topic_part, dist_Fx_predInit_topic, dist_Fy_predInit_topic, dist_Fz_predInit_topic, dist_Fx_data_topic, dist_Fy_data_topic, dist_Fz_data_topic;
bool online_ref_yaw;
bool control_stop;
bool use_dist_estimates;

double m_in, g_in;
Eigen::VectorXd Uref_in(NMPC_NU);
Eigen::VectorXd W_in(NMPC_NY);

int print_flag_offboard = 1, print_flag_arm = 1, 
print_flag_altctl = 1, print_flag_traj_finished = 0;

Eigen::Vector3d ref_trajectory, ref_velocity;
double ref_yaw_rad;
int ref_traj_type;
double t, t_cc_loop;

std::vector<double> pos_ref;
std::vector<double> current_pos_att;
std::vector<double> current_vel_rate;
std::vector<double> current_states;

struct _dist_struct
{
	bool predInit;
	int print_predInit = 1;
	std::vector<double> data;
	std::vector<double> data_zeros;
} dist_Fx, dist_Fy, dist_Fz;

#endif

