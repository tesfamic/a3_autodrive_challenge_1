#ifndef MISSIONCONTROL_WPT_FOLLOW_H
#define MISSIONCONTROL_WPT_FOLLOW_H
#include "ros/ros.h"
#include "aggies_lib/aggies_util.h"
#include "aggies_lib/aggies_param.h"


class WpFollower{ 

    private:
        int counter_;
        int current_index_;
        int nextwp_index_;

        std::vector<ExtendedWaypoint> route_;
        ExtendedWaypoint cur_location_;
        ExtendedWaypoint next_location_;
        ExtendedWaypoint prev_location_;
        ExtendedWaypoint dest_location_;
        double angle_error_;
        double target_vel_;
        //int steer_direction_;
        int num_pts_;

        double cur_steering_;
        double yaw_;
        double cur_throttle_;
        double azimuth_;
        double velocity_;

        UtilMethods util_;
        void FindNextWaypointIndex();
        
    public:
        ~WpFollower();
         WpFollower();
         void Reset();
         void SetWaypoints(std::vector<ExtendedWaypoint>& route);
         void Follow();//ExtendedWaypoint cur_loc,double azimuth);    
         int GetCurrenWpIndex(ExtendedWaypoint cur_loc,double azimuth);  
         int GetNextWpIndex();
         void GetControlVals(int turn,double &ang,double &speed);       
};

#endif //MISSIONCONTROL_WPT_FOLLOW_H