
#ifndef AGGIESAUTO_MISSIONCONTROL_TRAJECTORYPLANNER_H_
#define AGGIESAUTO_MISSIONCONTROL_TRAJECTORYPLANNER_H_

// #include "route_planner.h"
#include "aggies_lib/aggies_param.h"
#include "aggies_lib/aggies_util.h"
#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>

class TrajectoryPlanner{

    private:
        std::vector<ExtendedWaypoint> route_;
        std::vector<ExtendedWaypoint> modified_pts_;
        std::vector<ExtendedWaypoint> traj_xy_;

        int closeset_wp_indx_;
        ExtendedWaypoint current_pt_,prev_pt_;
        ExtendedWaypoint ref_pt_;
        WaypointXY ref_pt_xy_;
        std::vector<WaypointXY> trajectory_pts_;
        double azimuth_;
        UtilMethods util_;

        State vehicle_state_;
        double stop_distance_;

        void InterpolateWayPoints();
        void GenerateSpeedProfile();
        WaypointXY ToXyCoordinate(ExtendedWaypoint pt);
        void SelectWaypoints();
        void UpdateClosestPointIndex();
        //helper
        double EuclidDist(WaypointXY p1,WaypointXY p2);

    public:
        TrajectoryPlanner();
        TrajectoryPlanner(const std::vector<ExtendedWaypoint>& route,
                          ExtendedWaypoint& start_pt);

        ~TrajectoryPlanner();
        void Reset();
        void SetRoute(const std::vector<ExtendedWaypoint>& route, ExtendedWaypoint& start_pt);
        std::vector<WaypointXY> GetTrajectory(ExtendedWaypoint cur_loc,double azimuth);//, 
                                              //State v_state, double d_stop);
       std::vector<WaypointXY> GetModifiedTrajectory(ExtendedWaypoint cur_loc,double azimuth,int stop_pt_index);
       int GetClosestWaypointIndex();
       int GetGoalWaypointIndex();
};

#endif  //AGGIESAUTO_MISSIONCONTROL_TRAJECTORYPLANNER_H_
