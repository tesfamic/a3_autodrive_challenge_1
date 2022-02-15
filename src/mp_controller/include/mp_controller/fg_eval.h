#ifndef FGEVAL_H
#define FGEVAL_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
#include "aggies_lib/aggies_param.h"
#include "aggies_msgs/PathData.h"

using CppAD::AD;
using Eigen::VectorXd;

class FG_eval {
 public:
    // Fitted polynomial coefficients
    VectorXd coeffs;
    double v_ref; //reference velocity
    // double L; //wheel base
    // double Lr; //length b/n cg and front wheel axle

    FG_eval(VectorXd coeffs, double v_ref);//, double L, double Lr);

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars);
};
#endif //FGEVAL_H
