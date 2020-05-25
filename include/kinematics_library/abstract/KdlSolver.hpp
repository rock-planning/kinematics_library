#ifndef _KDLSOLVER_HPP_
#define _KDLSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include<iostream>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

#include "kinematics_library/abstract/AbstractKinematics.hpp"

namespace kinematics_library
{
    
    
struct KdlConfig
{
    KdlConfig() : max_iteration(150), eps(1e-6){}

    // Maximum iteration allowed for numerical solver.
    unsigned int max_iteration;
    // stopping criteria for the numerical solver. Stop the solver, if the error is below the eps.              
    double eps;
};

/**
 * @class AbstractKdlSolver
 * @brief Kinematics solvers using KDL.
 */
class KdlSolver : public AbstractKinematics
{

    public:
        /**
        * @brief  constructor
        */
        KdlSolver( const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree,
                   const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain);

        /**
        * @brief  destructor
        */
        virtual ~KdlSolver();

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
        bool solveFK(const base::samples::Joints &joint_angles,
                     base::samples::RigidBodyState &fk_pose,
                     KinematicsStatus &solver_status);
        
        void getChainSegementPose(const base::samples::Joints &joint_angles,  std::vector<KDL::Frame> &segement_pose);

    private:
        void getJointLimits(KDL::JntArray &min_jtLimits, KDL::JntArray &max_jtLimits);

        std::vector< std::pair<double, double> > jts_limits_;
        KDL::Chain kdl_kinematic_chain_;
        KDL::ChainFkSolverPos_recursive *fk_solverPos_;
        KDL::ChainIkSolverPos_NR_JL *ik_solverPosJL_;
        KDL::ChainIkSolverVel_pinv *ik_solverVelPinv_;        
        KDL::JntArray kdl_jtArray_, kdl_ik_jtArray_;
        KDL::JntArray min_jtLimits_, max_jtLimits_;
        KDL::Frame kdl_frame_;        
        unsigned int maxiter_;
        double eps_;
        
        KdlConfig kdl_config_;
};


}
#endif





