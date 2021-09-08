//
// Created by jonas on 7/22/21.
//
#include "state_machine.h"

#include <utility>

state_machine::state_machine(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
        nh_(nh), nh_private_(nh_private) {
    // Ros Publisher:
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // Ros Subscribers:
    current_position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1,
                                          &state_machine::poseCallback_local, this,
                                          ros::TransportHints().tcpNoDelay());

    current_position_global_sub_ = nh_.subscribe("/mavros/global_position/loca", 1,
                                                 &state_machine::poseCallback_global, this,
                                                 ros::TransportHints().tcpNoDelay());

    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 1, &state_machine::stateCallback, this,
                                                   ros::TransportHints().tcpNoDelay());
    // Ros Services
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    // Action timers:
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &state_machine::cmdloopCallback,
                                     this);  // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(1), &state_machine::statusloopCallback,
                                        this);  // Define timer for constant loop rate
    // PARAMETERS:
    nh_private_.param<bool>("verbose", verbose_, false);
    nh_private_.param<bool>("do_takeoff", do_takeoff_, false);
    nh_private_.param<string>("server_api", server_api_, "http://127.0.0.1:5000/inspection/mission_info");
}

state_machine::~state_machine() = default;


void state_machine::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
}

void state_machine::poseCallback_local(const geometry_msgs::PoseStamped &msg) {
    if (!received_home_pose) {
        received_home_pose = true;
        home_pose_ = msg.pose;
        ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
    }
    if (verbose_) {
        ROS_INFO("LOCAL ODOM");
        ROS_INFO("Seq: [%d]", msg.header.seq);
        ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg.pose.position.x, msg.pose.position.y,
                 msg.pose.position.z);
        ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg.pose.orientation.x,
                 msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    }
    current_pose_ = msg;
    current_rpy_ = pose2rpy(current_pose_.pose);
}

void state_machine::poseCallback_global(const nav_msgs::Odometry::ConstPtr &msg) const {
    // TODO:: Make global info work
//    if (!received_home_pose) {
//        received_home_pose = true;
//        home_pose_ = msg->pose.pose;
//        ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
//    }
    if (verbose_) {
        ROS_INFO("GLOBAL ODOM");
        ROS_INFO("Seq: [%d]", msg->header.seq);
        ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y,
                 msg->pose.pose.position.z);
        ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
    }

//    current_pose_ = *msg;
}

void state_machine::cmdloopCallback(const ros::TimerEvent &event) {
    // State machine
    switch (node_state) {
        case WAITING_FOR_HOME_POSE:
            print_state("STATE_MACHINE::WAITING_FOR_HOME_POSE");
            waitForPredicate(&received_home_pose, "Waiting for home pose...");
            ROS_INFO("Got pose! Drone Ready to be armed.");
            if (do_takeoff_) {
                node_state = TAKEOFF;
            } else if (waypointList.empty()) {
                node_state = MISSION_COMPUTE;
            } else {
                node_state = MISSION_EXECUTION;
            }
            break;
        case TAKEOFF: {
            print_state("STATE_MACHINE::TAKEOFF");
            if (init_drone()) {
                ros::Rate pause(2.0);
                ROS_INFO_STREAM("Waiting for takeoff...");
                while (ros::ok() && abs(home_pose_.position.z - current_pose_.pose.position.z) <= 1.5) {
                    ros::spinOnce();
                    pause.sleep();
                }
//                bool pred = abs(home_pose_.position.z - current_pose_.position.z) > 2;
//                waitForPredicate(&pred, "Waiting for takeoff...");
                // Compute new state
                if (waypointList.empty()) {
                    node_state = MISSION_COMPUTE;
                } else {
                    node_state = MISSION_EXECUTION;
                }
            }
            break;
        }
        case MISSION_COMPUTE: {
            print_state("STATE_MACHINE::MISSION_COMPUTE");

            // TODO:: Get to wind-turbine
            // 1. Determine what wind-turbine needs to be inspected
            // *** Through API or a ros topic.
            // 2. Get GPS information and dimensions of all wind-turbines in the farm.
            // *** Through API or a ros topic.
            // 3. Get GPS information and dimensions of ships in the area
            // *** Through API or a ros topic.
            // 4. Compute safe route to target wind-turbine.
            // *** Any planning algorithm
            // *** Use wind-turbine dimensions to compute unsafe zones


            // TODO:: Inspect blades
            // 1. Get information on this turbine
            // *** Global position of turbine
            // *** Height of the turbine
            // *** Dimension of the blades
            // *** wind direction ??? To get an idea of the heading of the wind-turbine???
            // 2. Based on information above, compute mission to get

            // TODO:: Only for debugging
            waypointList.push(generatePose(0, 0, 90, 0, 0, 90.0));
            waypointList.push(generatePose(-300, 200, 90, 0, 0, 180.0));
            waypointList.push(generatePose(-500, 400, 90, 0, 0, 270.0));
            node_state = MISSION_EXECUTION;
            break;
        }
        case MISSION_EXECUTION: {
            // Sanity check
            if (waypointList.empty() && target_pose_.pose == empty_pose_msg_) {
                node_state = MISSION_COMPUTE;
                break;
            }
            print_state("STATE_MACHINE::MISSION_EXECUTION");
            // Check if at the target location
            if (target_pose_.pose == empty_pose_msg_ || check_waypoint_reached()) {
                // Check if done:
                if (waypointList.empty()) {
                    target_pose_.pose = empty_pose_msg_;
                    node_state = MISSION_COMPUTE;
                    break;
                }

                target_pose_.header.stamp = ros::Time::now();
                target_pose_.pose = waypointList.front();
                target_rpy_ = pose2rpy(target_pose_.pose);
                waypointList.pop();
                local_pos_pub_.publish(target_pose_);
            }
            ROS_INFO("x: %f, y: %f, z: %f", target_pose_.pose.position.x, target_pose_.pose.position.y,
                     target_pose_.pose.position.z);

//            Eigen::Vector3d desired_acc;
//            if (feedthrough_enable_) {
//                desired_acc = targetAcc_;
//            } else {
//                desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
//            }
//            computeBodyRateCmd(cmdBodyRate_, desired_acc);
//            pubReferencePose(targetPos_, q_des);
//            pubRateCommands(cmdBodyRate_, q_des);
//            appendPoseHistory();
//            pubPoseHistory();
            break;
        }

        case LANDING: {
//            geometry_msgs::PoseStamped landingmsg;
//            landingmsg.header.stamp = ros::Time::now();
//            landingmsg.pose = home_pose_;
//            landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
//            target_pose_pub_.publish(landingmsg);
//            node_state = LANDED;
//            ros::spinOnce();
            break;
        }
        case LANDED:
//            ROS_INFO("Landed. Please set to position control and disarm.");
//            cmdloop_timer_.stop();
            break;
    }
}


void state_machine::statusloopCallback(const ros::TimerEvent &event) {
    // TODO:: Check if this is save to always force the drone to go OFFBOARD mode
    // TODO:: Make an error state to force this loop to not run.
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
            ROS_INFO("Offboard enabled");
        }
        last_request_ = ros::Time::now();
    } else {
        if (ros::Time::now() - last_request_ > ros::Duration(3.0)) {
            try {
                // you can pass http::InternetProtocol::V6 to Request to make an IPv6 request
                http::Request request{server_api_};

                // send a get request
                const auto response = request.send("GET");
                auto msg = std::string{response.body.begin(), response.body.end()};
                mission.Load(msg);
                windfarm_map_.clear();
                for (const auto &j : mission.ObjectRange()) {
                    windfarm_map_.emplace_back(j.second); // Creates a WindTurbine object given the args
//                    std::cout << "Object[ " << j.first << " ] = " << j.second << "\n";
                }
                if (verbose_) {
                    for (const auto &wt : windfarm_map_) {
                        ROS_INFO("windfarm_map_[ %s ] = %s", wt.name.c_str(), wt.to_string().c_str());
                    }
                } else {
                    ROS_INFO("MSG response: %s",
                             (std::string{response.body.begin(), response.body.end()}).c_str()); // print the result
                }
            }
            catch (const std::exception &e) {
                ROS_INFO("Request failed, error: %s", e.what());
            }
        }
        if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(3.0))) {
            if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request_ = ros::Time::now();
        }
    }
}

bool state_machine::init_drone() {
    ROS_INFO("Takeoff drone");
    takeoff_cmd_.request.altitude = 300.0;
    while (!takeoff_cmd_.response.success && ros::ok()) {
        ros::Duration(.1).sleep();
        takeoff_client_.call(takeoff_cmd_);
    }
    return takeoff_cmd_.response.success;
}

void state_machine::print_state(const string &text) {
    if (last_text_ != text) {
        ROS_INFO("%s", text.c_str());
        last_text_ = text;
    }
}

bool state_machine::check_waypoint_reached(float pos_tolerance, float heading_tolerance_deg) {
    target_pose_.header.stamp = ros::Time::now();
    local_pos_pub_.publish(target_pose_);

    //check for correct position
    double deltaX = abs(target_pose_.pose.position.x - current_pose_.pose.position.x);
    double deltaY = abs(target_pose_.pose.position.y - current_pose_.pose.position.y);
    double deltaZ = abs(target_pose_.pose.position.z - current_pose_.pose.position.z);
    double distance_magnitude = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

    //check orientation
    double headingErr = abs(
            rad2degree(atan2(sin(target_rpy_.yaw - current_rpy_.yaw), cos(target_rpy_.yaw - current_rpy_.yaw))));
    if (verbose_) {
        ROS_INFO("=========== check_waypoint_reached ========================");
        ROS_INFO("distance_magnitude %f", distance_magnitude);
        ROS_INFO("current pose x %F y %f z %f", (current_pose_.pose.position.x), (current_pose_.pose.position.y),
                 (current_pose_.pose.position.z));
        ROS_INFO("waypoint pose x %F y %f z %f", target_pose_.pose.position.x, target_pose_.pose.position.y,
                 target_pose_.pose.position.z);
        ROS_INFO("current heading %f", rad2degree(current_rpy_.yaw));
        ROS_INFO("local_desired_heading_g %f", rad2degree(target_rpy_.yaw));
        ROS_INFO("current heading error %f", headingErr);
        ROS_INFO("=========================================================");
    }
    return distance_magnitude < 0.3 && headingErr < heading_tolerance_deg;
}

