
#include "route_planner.h"

RoutePlanner::RoutePlanner(){
    start_pt_ = ExtendedWaypoint({-1.0,-1.0});
    goal_pt_ = ExtendedWaypoint({-1.0,-1.0});
    
    start_path_id_ = -1;
    goal_path_id_ = -1;
    route_available_ = false;

    this->LoadMap();
}

RoutePlanner::RoutePlanner(ExtendedWaypoint& start_pt,ExtendedWaypoint& goal_pt){
    this->start_pt_ = start_pt;
    this->goal_pt_ = goal_pt;
    
    start_path_id_ = -1;
    goal_path_id_ = -1;

    short_path_.clear();
    final_route_.clear();
    //final_route_xy_.clear();
    
    route_available_ = false;

    this->LoadMap();
}

RoutePlanner::~RoutePlanner(){
    ////
}

void RoutePlanner::Reset(){
    start_pt_ = ExtendedWaypoint({-1.0,-1.0});
    goal_pt_ = ExtendedWaypoint({-1.0,-1.0});
    
    start_path_id_ = -1;
    goal_path_id_ = -1;

    short_path_.clear();
    final_route_.clear();
    //final_route_xy_.clear();
    
    route_available_ = false;
    route_available_ = false;

    this->LoadMap();
}
void RoutePlanner::SetInitialLocation(ExtendedWaypoint& pt){
    this->start_pt_ = pt;
}

void RoutePlanner::SetGoalLocation(ExtendedWaypoint& pt){
    this->goal_pt_ = pt;
}

void RoutePlanner::LoadMap(){
    std::ifstream nodes_file("map/nodes.json");
    std::ifstream lanes_file("map/lanes.json");
    //load node files
    nlohmann::json node_tmp,lane_tmp;
    lanes_.clear();
    nodes_.clear();
    try{
        nodes_file>>node_tmp;
        nodes_ = node_tmp["nodes"];
        std::cout<<"\n Nodes are loaded.";
    }catch(const std::exception& e){
        std::cout<<"\n Unable to read map data: 'nodes.json'";
        std::cout << e.what() << '\n';
    }
        
    //load lane files 
    try{
        lanes_file>>lane_tmp;
        lanes_ = lane_tmp["lanes"];
        std::cout<<"\n Lanes are loaded.";
    }catch(const std::exception& e){
        std::cout<<"\n Unable to read map data: 'lanes.json'";
        std::cout << e.what() << '\n';
    }
}

void RoutePlanner::GetNearestNode(const ExtendedWaypoint& ref_node,int nd_id, std::vector<int>& lnks){
    //std::cout<<"\n inside fn: get_nearest_node.";
    int near_node_id = -1;
    double min_dist = std::numeric_limits<double>::max();
    ExtendedWaypoint node_pt;    
    for(auto& node:nodes_){
        int node_id;
        node["id"].get_to(node_id);
        node["loc"][1].get_to(node_pt.lat);
        node["loc"][0].get_to(node_pt.lon);
        double dist = util_.HaversineDist(ref_node,node_pt);
        if(dist<min_dist){
            min_dist = dist;
            near_node_id = node_id;
            nd_id = node_id;
            lnks.clear();
            for(auto&lnk:node["links"]){
                lnks.push_back(int(lnk));
            }
        }
    }
}

int RoutePlanner::GetNearestPath(ExtendedWaypoint& ref_node,ExtendedWaypoint& min_dist_pt){
    //get the nearest node and links 
    int nearest_node_id;
    std::vector<int> link_ids;
    this->GetNearestNode(ref_node,nearest_node_id,link_ids);
    
    //get the link id and path id   
    int link_id = -1;
    int path_id = -1;
    double min_dist = std::numeric_limits<double>::max();
    ////////////
    ///
    ////////////
    return path_id;
}

int RoutePlanner::GetNearestPath2(ExtendedWaypoint& ref_node,int path_id_ex,ExtendedWaypoint& min_dist_pt){
    //get the nearest node first

    int path_id = -1;
    /////// 
    return path_id;
}

ExtendedWaypoint RoutePlanner::GetCoordinate(const int path_id){
    // std::cout<<"\n inside fn:get_coordinate ...";
    ExtendedWaypoint coord_out;
    ////////////
    return coord_out;
}

double RoutePlanner::GetCost(const ExtendedWaypoint& pt1,const ExtendedWaypoint& pt2){
    // std::cout<<"\n inside fn: get_cost ...";
    return util_.HaversineDist(pt1,pt2);
}

std::vector<int> RoutePlanner::Expand(const int cur_pt_id){
    std::vector<int> expanded;
    for(auto& path:lanes_){
        if(path["path_id"]==cur_pt_id){
            auto conn_ids = path["connected_to"];
            for(auto& id:conn_ids){
                expanded.push_back(int(id));
            }
        }
    }
    return expanded;
}

void RoutePlanner::AStarPlanner(const int start_node_id, const int goal_node_id){
    
    ExtendedWaypoint start_pt = this->GetCoordinate(start_node_id);
    ExtendedWaypoint goal_pt = this->GetCoordinate(goal_node_id);

    double init_cost = this->GetCost(start_pt,goal_pt);
    
    std::vector<Tuple> frontier2;
    frontier2.push_back({init_cost,start_node_id});

    std::map<int,int> parent_tree;
    parent_tree[start_node_id] = 0;

    std::map<int,double> path_costs;
    path_costs[start_node_id] = 0.0;

    Tuple state_val = frontier2[0];
    int curr_node_id = state_val.id;
    while(!frontier2.empty()){
        //sort frontier2 based on cost
        std::sort(frontier2.begin(),frontier2.end(),
                    [](const Tuple &lhs,const Tuple &rhs){return lhs.cost>rhs.cost;});
        state_val = frontier2.back();
        curr_node_id = state_val.id;
        frontier2.pop_back(); //remove last element
        if(curr_node_id==goal_node_id){
            std::cout<<"\n Goal reached.";
            break;
        }

        std::vector<int> next_node_ids = Expand(curr_node_id);
        for(auto next_id:next_node_ids){
            if(next_id>0){
                ExtendedWaypoint cur_node_pt = GetCoordinate(curr_node_id);
                ExtendedWaypoint next_node_pt = GetCoordinate(next_id);

                //compute the path cost (g)
                double g = this->GetCost(cur_node_pt,next_node_pt) + path_costs[curr_node_id];
                bool inpath_costs=false;
                for(auto path_cost:path_costs){
                    if(next_id == path_cost.first){
                        inpath_costs = true;
                    }
                }                
                if((inpath_costs==false )||( g<path_costs[next_id])){
                    //calculate the heuristic value
                    double h = this->GetCost(next_node_pt,goal_pt);
                    double f = g + h;
                    //put next state in frontier
                    frontier2.push_back({f,next_id});
                    //update g_cost 
                    path_costs[next_id] = g;
                    parent_tree[next_id] = curr_node_id;
                }
            }
        }
    }
    bool path_found = false;
    if(curr_node_id != goal_node_id){
        path_found = false;
        std::cout<<"\n Unable to reach goal point.\n";
    }else{
        int cur_id = goal_node_id;
        short_path_.clear();
        while(cur_id != start_node_id){
            short_path_.push_back(cur_id);
            cur_id = parent_tree[cur_id];
        }
        short_path_.push_back(start_node_id);
        std::reverse(short_path_.begin(),short_path_.end());
        path_found = true;
    }
    // if(path_found){
    //     std::cout<<"\n Short path found.";
    //     // for(auto path:short_path_){
    //     //     std::cout<<" "<<path<<",";
    //     // }
    //     // std::cout<<"\n";
    // }
}


void RoutePlanner::PlanRoute(){

    int start_pt_id = this->GetNearestPath(this->start_pt_,car_start_pt_);
    int goal_pt_id = this->GetNearestPath(this->goal_pt_,car_goal_pt_);

    // route_available_ = (start_pt_id>0) & (goal_pt_id>0);

    // print the messages-useful to see what's happening 
    if(start_pt_id<0){
        std::cout<<"\n --> Error: Current (GPS) point is not available. \n    Check GPS unit. \n";
        route_available_ = false;
    }
    if(goal_pt_id<0){
        std::cout<<"\n --> Error: Unable to find destination point. \n   Change the destination point.\n";
        route_available_ = false;
    }
    if(start_pt_id>0 && goal_pt_id>0){
        this->AStarPlanner(start_pt_id,goal_pt_id);

        if(short_path_.size()>0){
            //continue with route construction
            this->ConstructRoute();
            route_available_ = true;
        }else{
            //try to find another goal point
            std::cout<<" \n ... second attempt to find goal point...\n";
            goal_pt_id = this->GetNearestPath2(this->goal_pt_,goal_pt_id,car_goal_pt_);
            if(short_path_.size()>0){
                //construct route
                this->ConstructRoute();
                route_available_ = true;
            }else{
                route_available_ = false;
                std::cout<<"\n --> Unable to find route. Select another destination point.\n";
            }
        }
    }
}

void RoutePlanner::ConstructRoute(){
    int start_id = short_path_[0];
    int goal_id = short_path_.back();

    //clear the route data first
    final_route_.clear();
    left_bound_.clear();
    right_bound_.clear();
    /////// removed ....
    ////////////////////////////
    std::cout<<"\n route construction completed.\n";
}

