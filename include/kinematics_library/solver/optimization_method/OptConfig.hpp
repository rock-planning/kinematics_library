#ifndef _OPT_CONFIG_HPP_
#define _OPT_CONFIG_HPP_


#include <iostream>

namespace kinematics_library
{

struct CostsWeight
{
    double ik;    
    double position;    
};

struct OptParamConfig
{
    double joint_movement_weight;
    double max_time;    // time in seconds. The opt solver will be running till max time.
    double abs_tol;     // absolute tolerance on optimization parameter
    double rel_tol;     // relative tolerance on optimization parameter
    double min_cost;    // minimum cost. Below this cost value, the problem is considered as solved.
};
}
#endif