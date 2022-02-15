#include <ros/ros.h>
#include <iostream>
#include "detector.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "lane_detector");

	Detector detect;
    
	ros::spin();

	return 0;
}