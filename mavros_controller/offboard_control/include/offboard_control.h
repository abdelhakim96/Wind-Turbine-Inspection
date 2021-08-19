//
// Created by jonas on 7/22/21.
//

#ifndef MAVROS_CONTROLLERS_OFFBOARD_CONTROL_H
#define MAVROS_CONTROLLERS_OFFBOARD_CONTROL_H

#include <cstdlib>
#include <queue>

#include <Eigen/Dense>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include "HTTPRequest.hpp"
#include "simple_json.hpp"
#include "wind_turbine.hpp"
#include "common.h"

using namespace std;
using namespace Eigen;

class offboard_control {
public:
    offboard_control(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    virtual ~offboard_control();


private:
    bool verbose_;
    bool do_takeoff_;
    string server_api_;
    string last_text_;
    simple_json::JSON mission;
    vector<WindTurbine> windfarm_map_;
    // Drone states
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    rpy_struct current_rpy_;
//    nav_msgs::Odometry current_pose_;

    geometry_msgs::Pose correction_vector_goal;
    geometry_msgs::Point local_offset_pose_goal;
    geometry_msgs::PoseStamped waypoint_goal;
    geometry_msgs::TwistStamped move;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::CommandTOL takeoff_cmd_;
    enum FlightState {
        WAITING_FOR_HOME_POSE, TAKEOFF, MISSION_COMPUTE, MISSION_EXECUTION, LANDING, LANDED
    } node_state = WAITING_FOR_HOME_POSE;

    template<class T>
    void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
        ros::Rate pause(hz);
        ROS_INFO_STREAM(msg);
        while (ros::ok() && !(*pred)) {
            ros::spinOnce();
            pause.sleep();
        }
    };

    geometry_msgs::Pose home_pose_;
    bool received_home_pose{};
    // Mission:
    queue<geometry_msgs::Pose> waypointList;
    queue<geometry_msgs::Pose> waypointListcurve;
    geometry_msgs::Pose empty_pose_msg_;
    geometry_msgs::PoseStamped target_pose_;
    rpy_struct target_rpy_;

    // Ros stuff
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::ServiceClient set_mode_client_, takeoff_client_, command_client_, arming_client_, land_client_;
    ros::Publisher local_pos_pub_, vel_pub_;
    ros::Subscriber current_pos_, state_sub_, goal_pose_sub_, current_position_sub_, current_position_global_sub_;
    ros::Timer cmdloop_timer_, statusloop_timer_;
    ros::Time last_request_, reference_request_now_, reference_request_last_;

    void stateCallback(const mavros_msgs::State::ConstPtr &msg);

    void poseCallback_local(const geometry_msgs::PoseStamped &msg);

    void poseCallback_global(const nav_msgs::Odometry::ConstPtr &msg) const;

    void cmdloopCallback(const ros::TimerEvent &event);

    void statusloopCallback(const ros::TimerEvent &event);

    bool init_drone();

    bool check_waypoint_reached(float pos_tolerance = 0.3, float heading_tolerance_deg = 2);

    void print_state(const string &text);

};


#endif //MAVROS_CONTROLLERS_OFFBOARD_CONTROL_H
