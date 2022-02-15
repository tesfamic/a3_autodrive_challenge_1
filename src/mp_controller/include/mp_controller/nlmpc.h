#ifndef NLMPC_H
#define NLMPC_H

#include "ros/ros.h"
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "aggies_msgs/PathData.h" //custom message
#include "aggies_msgs/CarCtrl.h"
#include "aggies_msgs/CarFeedback.h"
#include "aggies_msgs/ControlData.h"
#include "aggies_lib/aggies_util.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include <iostream>
#include <fstream>

#include "fg_eval.h"

class NLMPC {
    ros::NodeHandle n_;
    ros::Publisher control_pub_;     // publishes all the controll data
    ros::Publisher mpc_input_traj_pub_; //publishes the input trajectory (2nd order poly)
    ros::Publisher mpc_trajectory_pub_;   //output/tracked trajectory
    
    ros::Subscriber trajectory_sub_; //subscribes to waypoint trajectory
    ros::Subscriber gps_feedback_sub_; //subscriber to GPS /inspva topic: lat, lon, azimuth, ...
    ros::Subscriber vel_feedback_sub_; //feedback from GPS unit    
    ros::Subscriber current_orientation_sub_; //
    ros::Subscriber car_state_feedback_sub_; //
    ros::Subscriber odom_val_sub_;

    double velocity_;
    double cur_steering_;
    double cur_throttle_;

    UtilMethods utils_;

    std::vector<double> steering_;
    std::vector<double> throttle_;
    std::vector<double> x_sol_;
    std::vector<double> y_sol_;
    
    bool estim_started_;
    WaypointXY prev_pose_;
    double prev_time_;
    double prev_speed_;
    
 public:
    double latency;
    double v_ref; //reference velocity
    
    NLMPC();

    ~NLMPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    std::vector<double> Solve(const Eigen::VectorXd &state, 
                            const Eigen::VectorXd &coeffs);

   void TrajectCallback(const aggies_msgs::PathData msg);
   void CarStateCallback(const aggies_msgs::CarFeedback::ConstPtr& feedback);
   void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu);   
   void VelocityCallback(const novatel_gps_msgs::NovatelVelocity msg);
   void OdomValCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

};

#endif // NLMPC_H
