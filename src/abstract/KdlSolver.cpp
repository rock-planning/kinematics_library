#include <abstract/KdlSolver.hpp>

namespace kinematics_library
{

KdlSolver::KdlSolver(const KinematicsConfig &kinematics_config, const std::vector< std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree,
                     const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain, const unsigned int max_iter, const double eps): jts_limits_(jts_limits), maxiter_(max_iter), eps_(eps)
{
    kdl_tree_         = kdl_tree;
    kdl_chain_        = kdl_chain;
    maxiter_          = kinematics_config.max_iteration;
    eps_              = kinematics_config.eps;
    fk_solverPos_     = new KDL::ChainFkSolverPos_recursive(kdl_kinematic_chain);
    ik_solverVelPinv_ = new KDL::ChainIkSolverVel_pinv(kdl_kinematic_chain);

    assign_variables(kinematics_config, kdl_chain_);

    getJointLimits(min_jtLimits_, max_jtLimits_);       

    ik_solverPosJL_  = new KDL::ChainIkSolverPos_NR_JL(kdl_kinematic_chain, min_jtLimits_, max_jtLimits_, *fk_solverPos_, *ik_solverVelPinv_, maxiter_, eps_);

    kdl_jtArray_.data.resize(number_of_joints_);
    kdl_ik_jtArray_.data.resize(number_of_joints_);	

}

KdlSolver::~KdlSolver()
{
    if (fk_solverPos_)
    {
        fk_solverPos_ = NULL;
        delete fk_solverPos_;
    }

    if (ik_solverPosJL_)
    {
        ik_solverPosJL_ = NULL;
        delete ik_solverPosJL_;
    }

    if (ik_solverVelPinv_)
    {
        ik_solverVelPinv_ = NULL;
        delete ik_solverVelPinv_;
    }
}

bool KdlSolver::solveIK(const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status, std::vector<base::commands::Joints> &solution,
                        KinematicsStatus &solver_status)
{
    convertPoseBetweenDifferentFrames(kdl_tree_, target_pose, kinematic_pose_);

    getKinematicJoints(kdl_chain_, joint_status, jt_names_, current_jt_status_);

    convertVectorToKDLArray(current_jt_status_, kdl_jtArray_);
    rbsToKdl(kinematic_pose_, kdl_frame_);

    int res = ik_solverPosJL_->CartToJnt(kdl_jtArray_, kdl_frame_, kdl_ik_jtArray_);
    
    solution.resize(1);
    
    if( res >= 0)
    {
        convertKDLArrayToBaseJoints(kdl_ik_jtArray_, solution[0]);
        solution[0].names = jt_names_;
        solver_status.statuscode = KinematicsStatus::IK_FOUND;
        return true;
    }
    else if(res == -3)
    {
        solver_status.statuscode = KinematicsStatus::IK_TIMEOUT;
        return false;
    }
    else
    {
        solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
        return false;
    }
}

bool KdlSolver::solveFK(const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status)
{
    getKinematicJoints(kdl_chain_, joint_angles, jt_names_, current_jt_status_);

    convertVectorToKDLArray(current_jt_status_, kdl_jtArray_);

    if(fk_solverPos_->JntToCart(kdl_jtArray_, kdl_frame_) >= 0)
    {
        kdlToRbs(kdl_frame_, kinematic_pose_);
        solver_status.statuscode = KinematicsStatus::FK_FOUND;
        convertPoseBetweenDifferentFrames(kdl_tree_, kinematic_pose_, fk_pose);
        return true;
    }

    solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
    return false;
}

void KdlSolver::getJointLimits(KDL::JntArray &min_jtLimits, KDL::JntArray &max_jtLimits)
{
    min_jtLimits.resize(number_of_joints_);
    max_jtLimits.resize(number_of_joints_);

    for(std::size_t i = 0; i < number_of_joints_; i++)
    {
        min_jtLimits(i) = jts_limits_.at(i).first;
        max_jtLimits(i) = jts_limits_.at(i).second;
    }
}
}
