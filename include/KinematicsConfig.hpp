#ifndef _KINEMATICSCONFIG_HPP_
#define _KINEMATICSCONFIG_HPP_

#include <string>
#include <base/samples/RigidBodyState.hpp>

namespace kinematics_library
{

enum Robot
{
    ARTEMIS,
    KUKA,
    UNKNOWN
};
    
    
enum KinematicSolver
{
    IKFAST,
    CUSTOM_SOLVER,
    KDL
};



/**
 *  Contains the configuration for kinematics.
 */
struct KinematicsConfig
{
    KinematicsConfig() : kinematic_solver(KDL), robot(UNKNOWN)
                              {}

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
    // pick a robot
    enum Robot robot;
    
};

struct KinematicsStatus
{
    enum StatusCode
    {
	KDL_CHAIN_FAILED,
	KDL_INITIALISATION_FAILED,
	URDF_FAILED,
	NO_KINEMATIC_SOLVER_FOUND,
        IK_FOUND,
        FK_FOUND,
        NO_IK_SOLUTION,
        NO_FK_SOLUTION,
        IK_TIMEOUT,       
        IK_JOINTLIMITS_VIOLATED
    }statuscode;

};

} // end namespace 

#endif
