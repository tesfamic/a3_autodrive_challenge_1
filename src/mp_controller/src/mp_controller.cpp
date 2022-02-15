
#include <iostream>
#include "nlmpc.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "mp_controller");
    NLMPC controller;

    ros::spin();
   
    return 0;

}
