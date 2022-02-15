#ifndef AGGIESAUTO_MISSIONCONTROL_A3TIMER_H_
#define AGGIESAUTO_MISSIONCONTROL_A3TIMER_H_

#include <ros/ros.h>
#include <chrono>

class A3Timer{

    private:
        bool timer_init_;
        bool time_duration_over_;
        ros::Time init_time_;
        double time_duration_;
        bool time_duration_set_;

    public:
        A3Timer();
        ~A3Timer();
        void SetTimer();
        void ResetTimer();
        void SetTimerFor(double duration);
        double GetTimeDuration();
                
        bool IsInitialized();
        bool IsDurationInitialized();
        bool IsTimeDurationOver();
};

#endif //AGGIESAUTO_MISSIONCONTROL_A3TIMER_H_