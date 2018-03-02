#ifndef _KINEMATICSCONFIG_HPP_
#define _KINEMATICSCONFIG_HPP_

#include <string>
#include <base/samples/RigidBodyState.hpp>

namespace kinematics_library
{

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
    KinematicsConfig() : mKinematicSolver(KDL)
                              {}

    // kinematics solver: currently 3 types of solvers available
    enum KinematicSolver mKinematicSolver;
    // base name - from here inverse and forward kinematic will be calculated
    std::string mBaseName;
    //tip name -
    std::string mTipName;
    // joint weight. It is used for finding the optimal inverse kinematic solution
    std::vector <double> mJointsWeight;
    // minimum joints limits
    std::vector <double> mMinJointsLimits;
    // maximum joints limits
    std::vector <double> mMaxJointsLimits;
    // Number of Joints
    std::size_t mNumberOfJoints;
};

struct KinematicsStatus
{
    enum StatusCode
    {
        IK_FOUND,
        FK_FOUND,
        NO_IK_SOLUTION,
        NO_FK_SOLUTION,
        IK_TIMEOUT,
        INVALID_LINK_NAME,
        IK_FORBID_CONSTRAINTS, //ik solver forbidden by constraints
        IK_JOINTLIMITS_VIOLATED,
		INVALID_STATE
    }statuscode;

};

} // end namespace 

#endif
