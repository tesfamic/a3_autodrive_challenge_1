#ifndef MISSIONCONTROL_MISSION_H
#define MISSIONCONTROL_MISSION_H

#include <ros/ros.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include "aggies_msgs/CarCtrl.h"
#include "aggies_msgs/CarFeedback.h"
#include "aggies_msgs/ButtonStates.h"
#include "aggies_msgs/ControlData.h"
#include "aggies_msgs/PathData.h"
#include "aggies_msgs/ObstacleCentroid.h"

#include "aggies_lib/aggies_util.h"
#include "aggies_lib/aggies_param.h"
#include "route_planner.h"
#include "traj_plan.h"
#include "trajectory_planner.h"
#include "wpt_follower.h"
#include "a3_timer.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <math.h>

class Mission{
  private:
    ros::NodeHandle nh_;
    ros::Publisher car_control_pub_;
    ros::Subscriber car_state_feedback_sub_;     
    ros::Subscriber btn_state_sub_;  
    ros::Subscriber control_val_sub_;
    ros::Subscriber velocity_val_sub_;

    //data from other nodes
    ros::Subscriber obstacle_detector_sub_; //from lidar
    ros::Subscriber lane_detector_sub_; 
    ros::Subscriber traffic_light_sub_;
    ros::Subscriber traffic_sign_sub_;
    ros::Subscriber pedestrian_detector_sub_;

    ros::Subscriber velocity_sub_;
    ros::Subscriber position_val_sub_; //position feedback 
    ros::Subscriber imu_val_sub_; //
    ros::Subscriber traj_off_sub_;

    ros::Publisher traj_pub_;
    ros::Publisher route_pub_;
    ros::Publisher goalpt_pub_;
    
    ////////////Routing related ....
    //RoutePlanner *route_planner_;
    //TrajectoryPlanner *traj_planner_;
    WpFollower wp_follower_;
    std::vector<WaypointXY> trajectory_;
    std::vector<ExtendedWaypoint> destinations_;
    ExtendedWaypoint dest_loc_;
    ExtendedWaypoint curr_loc_;
    ExtendedWaypoint ref_loc_;

    /////////////////
    TLight traffic_light_;
    TLight prev_tl_state_;
    TSign traffic_sign_;

    std::vector<geometry_msgs::Point> obstacles_;
    
    StopCause stop_caused_by_; 
    std::vector<std::pair<StopCause,double>> stop_causes_;
    bool required_to_stop_;
    bool stop_speed_reached_;
    WaypointXY stop_pt_;

    ////Obstacle 
    int8_t obstacle_pt_;   
    bool obstacle_detected_;
    double obstacle_distance_;
    double stop_distance_;   

    ///traffic light related
    // int8_t red_light_pt_;
    bool red_light_detected_;
    bool override_red_light_;
    bool hold_tl_detection_;
    double override_time_began_;
    double tl_distance_;
    ExtendedWaypoint tl_location_;

    ////stop sign related
    // int stop_sign_pt_;
    double stop_sign_distance_;
    bool stop_sign_detected_;
    WaypointXY stop_sign_coordinate_;
    ExtendedWaypoint stop_sign_location_;
    
    ////goal point 
    // int8_t dest_pt_;
    int dest_pt_index_;
    bool destination_reached_;
    //time
    A3Timer stop_sign_timer_;
    A3Timer red_light_timer_;
    A3Timer obstacle_stop_timer_;
    A3Timer dest_stop_timer_;
    A3Timer hold_tl_timer_;
    
    /////////
    bool parking_requested_;

    ///traj_plan
    TrajPlanner traj_planner_;
    std::vector<ExtendedWaypoint> modified_pts_;
    int prv_pt_indx_;
    double traj_shift_;
    int turn_state_;
    aggies_msgs::PathData lane_center_;

  public: 
    Mission();
    
    ~Mission();

    void FSM();
    void NormalState();
    
    void SetControl();
    void SetControl(double ang_error,double speed);
    
    void ApplyBrakeHold();
    void InitializeBrakeHoldState();
    void ChangeToDriveGear();
    void ManualState();
    
    void CarStateCallback(const aggies_msgs::CarFeedback::ConstPtr& car_feedback);   
    void ButtonStateCallback(const aggies_msgs::ButtonStates::ConstPtr& btn_state_msg);
    void ControlValueCallback(const aggies_msgs::ControlData control_msg);  
    
    void VelocityValCallback(const novatel_gps_msgs::NovatelVelocity::ConstPtr& vel_msg);
    void PositionValCallback(const novatel_gps_msgs::Inspva::ConstPtr& pos_msg);
    void ImuValCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    ////////////////////
    void ObstacleDetectorCallback(const aggies_msgs::PathData obst_msg);//ObstacleCentroid::ConstPtr& obst_msg);
    void TrafficLightCallback(const std_msgs::Int8::ConstPtr& tlight_msg);
    void TrafficSignCallback(const aggies_msgs::PathData::ConstPtr& sign_msg);    
    void LaneDetectorCallback(const aggies_msgs::PathData& lane_msg);
    void TrajOffsetCallback(const std_msgs::Int8::ConstPtr& tr_off);
    //void FreespaceDetectorCallback(const aggies_msgs::??);
    //void PedestrianDetectorCallback(const aggies_msgs::??);

    ////////////////////
    void StoppingHandler();
    void StopForDestination();
    void StopForObstacle();
    void StopForRedLight();
    void StopForStopSign();

    void StartState();
    void LoadDestinationLocation();
    void PerformRouting();
    void StopState();
    void StoppingState();
    void ParkedState();

    void CheckPerception();
    void DetermineClosestStop();
    void CheckCurrentSpeed();
    void PublishRoute();

    void AdjustVelocity();
    void AdjustAngle();
    void ModifyTurn();
    void GetClosestObstacle();
    void GetClosestStop();
    //double GetNearestTLstop();

 };
#endif //MISSIONCONTROL_MISSION_H