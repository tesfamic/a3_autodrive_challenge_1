#include <iostream>
#include <ros/ros.h>
#include "detector_cnn.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "cnn_lane_detector");

	CnnDetector lane;
	ros::spin();

	return 0;
}