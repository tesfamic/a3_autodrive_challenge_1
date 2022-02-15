
#ifndef AGGIESLIB_UTIL_H_
#define AGGIESLIB_UTIL_H_

#include <fstream>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
const double R_EARTH = 6371000;

struct Tuple{
    double cost;
    int id;
};

enum class TLight{
    kRed,
    kGreen, 
    kYellow,    
    kUnknown
};

enum class TSign{
    k25Mph,
    k20Mph,
    k15Mph,
    k10Mph,
    k5Mph,
    kStop,
    kLeftTurn,
    kRightTurn,
    kNoEntrance,
    kYield,
    kPedestrianXing,
    kDoNotEnter,
    kUnknown
};

enum class State{
    kManual=500, //start with 500 for manual,start=501,..
    kStart, 
    kInitial,
    kReady,
    kNormal,
    kStop,
    kStopping,
    kParked
};

enum class StopCause{
    kRedLight,
    kStopSign,
    kObstacle,
    kDestination,
    kUnknown
};

class UtilMethods{
    public:
        double ToRad(const double angle);
        double ToDeg(const double angle);
        double HaversineDist(const ExtendedWaypoint pt1, const ExtendedWaypoint pt2);
        double BearingAngle(ExtendedWaypoint pt1, ExtendedWaypoint pt2);
        int GetDirection(const double heading, const double bearing);
        bool AreEqual(const ExtendedWaypoint pt1, const ExtendedWaypoint pt2);
        double Polyeval(const Eigen::VectorXd &coeffs, double x);
        Eigen::VectorXd Polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);

};

#endif  // AGGIESLIB_UTIL_H_




