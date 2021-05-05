/***************************************************************************/
/*  An Optimization based kinematic solver. The inverse kinematic problem  */
/*  is formualted as a wieghted sum optimization problem. The objective    */
/*  functions are normalised using a parametric normalization function,    */
/*  which was proposed by Rakita et.al in                                  */
/*  RelaxedIK: Real-time Synthesis of Accurate andFeasible Robot Arm Motion*/
/*                                                                         */
/*  Author: Sankaranarayanan Natarajan                                     */
/*  DFKI - BREMEN 2021                                                     */
/***************************************************************************/

#ifndef _OPTSOLVER_HPP_
#define _OPTSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>
#include <iostream>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <nlopt.hpp>

#include "kinematics_library/solver/optimization_method/ProblemFormulation.hpp"

namespace kinematics_library
{
/**
 * @class OptSolver
 * @brief Kinematics solvers using TracIk.
 */
class OptSolver : public AbstractKinematics
{
    public:
        /**
        * @brief  constructor
        */
        OptSolver(  const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree, 
                    const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain);

        /**
        * @brief  destructor
        */
        ~OptSolver();

        bool loadKinematicConfig( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status);

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
        
        void getChainSegementPose(const base::samples::Joints &joint_angles,  std::vector<KDL::Frame> &segement_pose){}

        ProblemFormulation problem_formulation_;

    private:
        KDL::ChainFkSolverPos_recursive *fk_kdlsolver_pos_;
        KDL::JntArray kdl_jt_array_;
        KDL::Frame kdl_frame_;
        KDL::Chain kdl_kinematic_chain_;
        nlopt::opt nlopt_solver_;
        std::vector<double> opt_var_;
        std::vector<std::pair<double,double> > jts_limits_;
        ProblemParameters opt_param_;

        bool initialiseProblem(const ProblemParameters &problem_param);

        void getJointLimits(const std::vector<std::pair<double, double> > jts_limits, 
                            std::vector< double > &lower_limits, std::vector< double > &upper_limits);

};
}
#endif





