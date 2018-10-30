#ifndef ABSTRACTKINEMATIC_HPP_
#define ABSTRACTKINEMATIC_HPP_

#include <vector>
#include <boost/function.hpp>
#include <iostream>
#include <string>
#include <base/Eigen.hpp>
#include "KinematicsConfig.hpp"
#include "KinematicsHelper.hpp"

/** \file AbstractKinematics.hpp
*    \brief Abstract kinematics header.
*/

namespace kinematics_library
{

/**
 * @class AbstractKinematic
 * @brief Provides an abstract interface for kinematics solvers.
 */
class AbstractKinematics
{

public:
    /**
    * @brief  constructor
    */
    AbstractKinematics();
    /**
    * @brief  destructor
    */
    virtual ~AbstractKinematics();	
	
    /**
    * @brief Calculate the joint angles for a robot to reach a desired Pose.    
    * @param target_position Desired position for the target link
    * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
    * @param joint_status Contains current joint angles.
    * @param solution Inverse solution
    * @param solver_status Solution status or error code
    * @return true if a inverse solution was found or else return false
    */
    virtual bool solveIK( const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                            base::commands::Joints &solution, KinematicsStatus &solver_status) = 0;

    /**
    * @brief Calculate pose of a robot given its joint angles
    * @param joint_angles joint angles of the robot
    * @param fk_position fk position
    * @param fk_orientation fk orienation in quaternion
    * @param solver_status Solution status or error code
    * @return true if a forward solution was found or else return false
    */
    virtual bool solveFK( const base::samples::Joints &joint_status, base::samples::RigidBodyState &fk_pose,                        
                          KinematicsStatus &solver_status) = 0;

    bool solveIKRelatively( const base::samples::Joints &joint_angles, const base::samples::RigidBodyState &relative_pose,
                            base::commands::Joints &solution, KinematicsStatus &solver_status);

protected:
    base::samples::RigidBodyState kinematic_pose_;
    std::vector<double>current_jt_status_, ik_solution_;
    std::vector<std::string> jt_names_;
    std::size_t number_of_joints_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    void assign_variables(const KinematicsConfig &kinematics_config, const KDL::Chain &kdl_chain);

};

typedef std::shared_ptr<kinematics_library::AbstractKinematics> AbstractKinematicPtr;
//typedef boost::shared_ptr<const AbstractKinematic> AbstractKinematicConstPtr;

};

#endif
