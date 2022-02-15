
#include "a3_timer.h"

A3Timer::A3Timer(){
    timer_init_ = false;
    time_duration_set_ = false;
    time_duration_ = 0.0;
    init_time_ = ros::Time::now();
}
A3Timer::~A3Timer(){
    ResetTimer();
}

