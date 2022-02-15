#include <iostream>
#include "mission.h"


int main(int argc, char *argv[]){
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);        
   
    Mission plan;
      
    while(ros::ok()){
        plan.FSM();            
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout<<"\n Mission ended/interrupted."<<std::endl;
    
    return 0;
}