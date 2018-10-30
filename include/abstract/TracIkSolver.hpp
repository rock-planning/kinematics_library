#ifndef _TRACIKSOLVER_HPP_
#define _TRACIKSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include<iostream>

#include <trac_ik/trac_ik.hpp>

#include "AbstractKinematics.hpp"
#include "KinematicsHelper.hpp"

namespace kinematics_library
{

/**
 * @class TracIkSolver
 * @brief Kinematics solvers using TracIk.
 */
class TracIkSolver : public AbstractKinematics
{

    public:
        /**
        * @brief  constructor
        */
        TracIkSolver(const KinematicsConfig &kinematics_config, const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain);

        /**
        * @brief  destructor
        */
        ~TracIkSolver();

        /**
        * @brief Calculate the joint angles for a manipulator to reach a desired Pose.
        * @param target_position Desired position for the target link
        * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
        * @param joint_status Contains current joint angles.
        * @param solution Inverse solution
        * @param solver_status Solution status or error code
        * @return true if a inverse solution was found or else return false
        */
        bool solveIK(const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                    base::commands::Joints &solution, KinematicsStatus &solver_status);

        /**
        * @brief Calculate pose of a manipulator given its joint angles        
        * @param joint_angles joint angles of the manipulator
        * @param fk_position fk position
        * @param fk_orientation fk orienation in quaternion
        * @param solver_status Solution status or error code
        * @return true if a forward solution was found or else return false
        */
        bool solveFK(const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status);

    private:
        KDL::ChainFkSolverPos_recursive *fk_kdlsolver_pos_;

        KDL::Frame kdl_frame_;
        KDL::JntArray kdl_jt_array_, kdl_ik_jt_array_;
        std::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_;
        std::string base_link_, tip_link_;
        std::string  urdf_file_path_;
        unsigned int max_iter_;
        double eps_;
};
}
#endif





