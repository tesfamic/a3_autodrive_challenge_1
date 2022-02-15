#ifndef AGGIESAUTO_MISSIONCONTROL_ROUTEPLANNER_H_
#define AGGIESAUTO_MISSIONCONTROL_ROUTEPLANNER_H_

#include <fstream>
#include <iostream>
#include <iomanip>
#include <math.h>
#include "json.hpp"
#include "aggies_lib/aggies_util.h"
 
class RoutePlanner{

    private:
        nlohmann::json nodes_;
        nlohmann::json lanes_;

        UtilMethods util_;

        ExtendedWaypoint start_pt_; //poi begin location
        ExtendedWaypoint goal_pt_;  //poi goal location

        ExtendedWaypoint car_start_pt_; //actual vehicle location
        ExtendedWaypoint car_goal_pt_;  //feasible goal location

        int start_path_id_;
        int goal_path_id_;

        std::vector<int> short_path_;
        std::vector<ExtendedWaypoint> final_route_;
        std::vector<ExtendedWaypoint> left_bound_;
        std::vector<ExtendedWaypoint> right_bound_;
        // std::vector<int> turn_prop_;
        // std::vector<int> direc_prop_;
        // std::vector<ExtendedWaypoint> adas_;
        // std::vector<ExtendedWaypoint> stops_;

        bool route_available_;

        void LoadMap();
        void GetNearestNode(const ExtendedWaypoint& ref_node, int nd_id, std::vector<int>& lnks);
        int GetNearestPath(ExtendedWaypoint& ref_node, ExtendedWaypoint& min_dist_pt);
        int GetNearestPath2(ExtendedWaypoint& ref_node,int path_id_ex, ExtendedWaypoint& min_dist_pt);

        ExtendedWaypoint GetCoordinate(const int path_id);
        double GetCost(const ExtendedWaypoint &pt1,const ExtendedWaypoint& pt2);
        std::vector<int> Expand(const int cur_pt_id);
        void AStarPlanner(const int start_node_id, const int goal_node_id);

    public:
        
        RoutePlanner();
        RoutePlanner(ExtendedWaypoint& start_pt,ExtendedWaypoint& goal_pt);
        ~RoutePlanner();
        void Reset();
        void PlanRoute();
        std::vector<ExtendedWaypoint> GetRoute();
        void ConstructRoute();
        bool IsRouteAvailable();
        void SetInitialLocation(ExtendedWaypoint& pt);
        void SetGoalLocation(ExtendedWaypoint& pt);
        std::vector<ExtendedWaypoint> ToXyCoordinate(std::vector<ExtendedWaypoint>& in_pts);

        // std::vector<ExtendedWaypoint> GetLeftBoundary();
        // std::vector<ExtendedWaypoint> GetRightBoundary();
        // std::vector<int> GetTurnProp();
        // std::vector<int> GetDirectionProp();
        // std::vector<ExtendedWaypoint> GetAdasProp();
        // std::vector<ExtendedWaypoint> GetStopProp();

};
#endif //AGGIESAUTO_MISSIONCONTROL_ROUTEPLANNER_H_
