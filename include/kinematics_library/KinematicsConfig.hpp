#ifndef _KINEMATICSCONFIG_HPP_
#define _KINEMATICSCONFIG_HPP_

#include <string>
#include <base/samples/RigidBodyState.hpp>
#include <vector>

namespace kinematics_library
{   
   
enum KinematicSolver
{
    IKFAST,
    SRS,
    KDL,
    TRACIK
};

struct LinearConfig
{
    LinearConfig(): interpolation_velocity(0.1), sampling_time(0.01), position_tolerance_in_m(0.02) {}

    // Relative target: linear interpolation in m/s
    double interpolation_velocity;
    // Relative target: sampling time in s
    double sampling_time;
    // Relative target: minimum distance the interpolation should reach
    double position_tolerance_in_m;
};


/**
 *  Contains the configuration for kinematics.
 */
struct KinematicsConfig
{
    KinematicsConfig() : base_name(""), tip_name(""), kinematic_solver(KDL), solver_config_filename(""),
                         urdf_file(""), linear_relative_movement(false) {}


    // base name - from here inverse and forward kinematic will be calculated
    std::string base_name;
    //tip name -
    std::string tip_name;
    // kinematics solver: currently 3 types of solvers available
    enum KinematicSolver kinematic_solver;
    // please specify the absolute path where all the config is located
    std::string solver_config_abs_path;
    // config file name specific to the kinematic solver - only file name
    std::string solver_config_filename;
    //URDF file for the robot. Please give filename with absolute path
    std::string urdf_file;
    // move relative target linearly
    bool linear_relative_movement;
    // relative movement configuration
    LinearConfig linear_movement_config;
};

struct KinematicsStatus
{
    enum StatusCode
    {
        KDL_TREE_FAILED,
        KDL_CHAIN_FAILED,
        URDF_FAILED,
        NO_KINEMATIC_SOLVER_FOUND,
        IK_FOUND,
        FK_FOUND,
        NO_IK_SOLUTION,
        NO_FK_SOLUTION,
        IK_TIMEOUT,       
        IK_JOINTLIMITS_VIOLATED,
        IKFAST_LIB_NOT_AVAILABLE,
        IKFAST_FUNCTION_NOT_FOUND,
        INVALID_STATE
    }statuscode;

};

} // end namespace 

#endif
