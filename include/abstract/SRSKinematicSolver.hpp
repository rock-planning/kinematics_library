/***************************************************************************/
/*  An analytical kinematic solver for a 7-DOF Anthropomorphic Manipulator */
/*  (SRS type - Spherical - Revolute - Spherical Joints)                   */
/*                                                                         */
/*  This solver is based on the paper:                                     */
/*  "Analytical Inverse Kinematic Computation for 7-DOF Redundant          */
/*   Manipulators With Joint Limits and Its Application to Redundancy      */
/*   Resolution" - Masayuki Shimizu, Hiromu kakuya, Woo-Keun Yoon,         */
/*   Kosei Kitagaki, Kazuhiro Kosuge.                                      */
/*                                                                         */
/*                                                                         */
/*  Author: Sankaranarayanan Natarajan                                     */
/*  DFKI - BREMEN 2019                                                     */
/***************************************************************************/

#ifndef _SRSKINEMATICSOLVER_HPP_
#define _SRSKINEMATICSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include<iostream>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "AbstractKinematics.hpp"
#include "KinematicsHelper.hpp"

namespace kinematics_library
{

/**
 * @class SRSKinematicSolver
 * @brief An analytical kinematics solvers for an anthropomorphic manipulator.
 */
class SRSKinematicSolver : public AbstractKinematics
{

    public:
        /**
        * @brief  constructor
        */
        SRSKinematicSolver(const KinematicsConfig &kinematics_config,  const std::vector<std::pair<double, double> > &jts_limits, 
                           const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain);

        /**
        * @brief  destructor
        */
        ~SRSKinematicSolver();

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
                     std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status);

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
        std::string base_link_, tip_link_;
        std::string  urdf_file_path_;
        
        double dist_base_shoulder;              // DBS - distance between base to shoulder
        double dist_shoulder_elbow;             // DSE - distance between shoulder to elbow
        double dist_elbow_wrist;                // DEW - distance between elbow to wrist
        double dist_wrist_tool;                 // DWT - distance between wrist to tool
//         std::vector<double> l_bs, l_se, l_ew, l_wt;     // vector holding the link length
        std::vector<double> link_base_shoulder, link_shoulder_elbow, link_elbow_wrist, link_wrist_tool;     // vector holding the link length
        double link_offset[7];                              // link offsets

        double min_j1, max_j1;                          // minimum and maximum values for joint 1
        double min_j2, max_j2;                          // minimum and maximum values for joint 2
        double min_j3, max_j3;                          // minimum and maximum values for joint 3
        double min_j5, max_j5;                          // minimum and maximum values for joint 5
        double max_j6, min_j6;                          // minimum and maximum values for joint 6
        double max_j7, min_j7;                          // minimum and maximum values for joint 7

        std::vector< ArmAngle > feasible_psi;
        std::vector< ArmAngle > infeasible_psi;

};
}
#endif





