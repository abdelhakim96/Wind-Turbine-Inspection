//
// Created by jonas on 7/22/21.
//
#include "offboard_control.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mavros_controllers");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    offboard_control offboardControl(nh, nh_private);

    ros::spin();
    return 0;
}



