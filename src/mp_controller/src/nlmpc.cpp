
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include "nlmpc.h"

using CppAD::AD;
using Eigen::VectorXd;


NLMPC::NLMPC() {
  
}
NLMPC::~NLMPC() {}

