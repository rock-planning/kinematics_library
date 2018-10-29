#ifndef _KINEMATICSCONFIG_HPP_
#define _KINEMATICSCONFIG_HPP_

#include <string>
#include <base/samples/RigidBodyState.hpp>

namespace kinematics_library
{   
   
enum KinematicSolver
{
    IKFAST,    
    KDL,
    TRACIK
};

/**
 *  Contains the configuration for kinematics.
 */
struct KinematicsConfig
{
    KinematicsConfig() : kinematic_solver(KDL), base_name(""), tip_name(""),
			 number_of_joints(0), urdf_file(""), max_iteration(100),
			 timeout_sec(0), eps(0){}

    // kinematics solver: currently 3 types of solvers available
    enum KinematicSolver kinematic_solver;
    // base name - from here inverse and forward kinematic will be calculated
    std::string base_name;
    //tip name -
    std::string tip_name;
    // joint weight. It is used for finding the weighted optimal inverse kinematic solution, 
    // when the inverse solver gave more than one solution.
    std::vector <double> joints_weight;    
    // Number of Joints
    std::size_t number_of_joints;	
    //URDF file for the robot. Please give filename with absolute path
    std::string urdf_file;
    // Maximum iteration allowed for numerical solver.
    unsigned int max_iteration;
    // time out - used if trac ik is available
    double timeout_sec;
    // stopping criteria for the numerical solver. Stop the solver, if the error is below the eps.              
    double eps;
    // ikfast shared library absolute path
    std::string ikfast_lib;
    
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
