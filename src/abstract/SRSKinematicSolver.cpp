#include <abstract/SRSKinematicSolver.hpp>

namespace kinematics_library
{

SRSKinematicSolver::SRSKinematicSolver ( const KinematicsConfig &kinematics_config,  const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree, 
                                         q
                                         const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain )
{

    kdl_tree_       = kdl_tree;
    kdl_chain_      = kdl_chain;

    fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_kinematic_chain );

    assignVariables ( kinematics_config, kdl_chain_ );
    kdl_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    kdl_ik_jt_array_.resize ( kdl_chain_.getNrOfJoints() );

}

SRSKinematicSolver::~SRSKinematicSolver()
{
    if ( fk_kdlsolver_pos_ ) 
    {
        fk_kdlsolver_pos_ = NULL;
        delete fk_kdlsolver_pos_;
    }
}

bool SRSKinematicSolver::solveIK (const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                            std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status )
{
    convertPoseBetweenDifferentFrames ( kdl_tree_, target_pose, kinematic_pose_ );
    
    getKinematicJoints ( kdl_chain_, joint_status, jt_names_, current_jt_status_ );

    
}

bool SRSKinematicSolver::solveFK (const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status )
{
    getKinematicJoints ( kdl_chain_, joint_angles, jt_names_, current_jt_status_ );

    convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );

    if ( fk_kdlsolver_pos_->JntToCart ( kdl_jt_array_, kdl_frame_ ) >= 0 ) 
    {
        kdlToRbs ( kdl_frame_, kinematic_pose_ );
        solver_status.statuscode = KinematicsStatus::FK_FOUND;
        convertPoseBetweenDifferentFrames ( kdl_tree_, kinematic_pose_, fk_pose );
        return true;
    }

    solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
    return false;
}
}
