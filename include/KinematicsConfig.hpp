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
    KDL,
    TRACIK
};

enum TracIKSolverType
{
	SPEED = 0,
	DISTANCE,
	MANIP1,
	MANIP2
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
    KinematicsConfig() : kinematic_solver(KDL), base_name(""), tip_name(""),
                         urdf_file(""), max_iteration(100), timeout_sec(0.1), eps(0.001), tolerances(6, 0.0),
                         ikfast_lib(""), tracIKSolverType(SPEED), linear_relative_movement(false) {}

    // kinematics solver: currently 3 types of solvers available
    enum KinematicSolver kinematic_solver;
    // base name - from here inverse and forward kinematic will be calculated
    std::string base_name;
    //tip name -
    std::string tip_name;
    // joint weight. It is used for finding the weighted optimal inverse kinematic solution, 
    // when the inverse solver gave more than one solution.
    std::vector <double> joints_weight;    	
    //URDF file for the robot. Please give filename with absolute path
    std::string urdf_file;
    // Maximum iteration allowed for numerical solver.
    unsigned int max_iteration;
    // time out - used if trac ik is available
    double timeout_sec;
    // stopping criteria for the numerical solver. Stop the solver, if the error is below the eps.              
    double eps;
    //End Effector Pose Tolerance - used for setting the tolerances in trac_ik
    std::vector<double> tolerances;
    // ikfast shared library absolute path
    std::string ikfast_lib;
	// Type of solver for trac_ik- selecting speed gives the first ik soln selecting distance tries to minimize distance between current and solution in given time
	TracIKSolverType tracIKSolverType;
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
