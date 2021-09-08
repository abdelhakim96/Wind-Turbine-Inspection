//
// Created by jonas on 7/22/21.
//
#include "state_machine.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mavros_controllers");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    state_machine stateMachine(nh, nh_private);

    ros::spin();
    return 0;
}



