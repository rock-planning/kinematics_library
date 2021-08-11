#include "kinematics_library/HandleKinematicConfig.hpp"

namespace kinematics_library
{

KdlSolver::KdlSolver(const std::vector< std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree,
                     const KDL::Chain &kdl_kinematic_chain, const KDL::Chain &kdl_chain ):
                     jts_limits_(jts_limits)
{
    kdl_tree_  = kdl_tree;
    kdl_chain_ = kdl_chain;
    kdl_kinematic_chain_ = kdl_kinematic_chain;
}

KdlSolver::~KdlSolver()
{
    if (fk_solverPos_)
    {
        delete fk_solverPos_;
        fk_solverPos_ = NULL;        
    }

    if (ik_solverPosJL_)
    {
        delete ik_solverPosJL_;
        ik_solverPosJL_ = NULL;        
    }

    if (ik_solverVelPinv_)
    {
        delete ik_solverVelPinv_;
        ik_solverVelPinv_ = NULL;        
    }
}

bool KdlSolver::loadKinematicConfig( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status)
{    
    // assign the config
    YAML::Node input_config;
    // check whether the config could be loaded or not.
    if(!handle_kinematic_config::loadConfigFile(kinematics_config.solver_config_abs_path, kinematics_config.solver_config_filename, input_config))
    {
        LOG_ERROR("[KdlSolver]: Unable to load kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(), 
                    kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::NO_CONFIG_FILE;
        return false;
    }

    const YAML::Node& kdl_config_node = input_config["kdl_config"];
    if(!handle_kinematic_config::getKdlConfig(kdl_config_node, kdl_config_))
    {
        LOG_ERROR("[KdlSolver]: Unable to read kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(), 
                    kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::CONFIG_READ_ERROR;
        return false;
    }

    fk_solverPos_     = new KDL::ChainFkSolverPos_recursive(kdl_chain_);
    ik_solverVelPinv_ = new KDL::ChainIkSolverVel_pinv(kdl_chain_);

    assignVariables(kinematics_config, kdl_kinematic_chain_);

    getJointLimits(min_jtLimits_, max_jtLimits_);       

    ik_solverPosJL_  = new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, min_jtLimits_, max_jtLimits_, *fk_solverPos_, *ik_solverVelPinv_, 
                                                       kdl_config_.max_iteration, kdl_config_.eps);

    kdl_jtArray_.data.resize(number_of_joints_);
    kdl_ik_jtArray_.data.resize(number_of_joints_);

    kinematics_status.statuscode = KinematicsStatus::SUCCESS;
    return true;
}

bool KdlSolver::solveIK(const base::samples::RigidBodyState &target_pose, const base::samples::Joints &joint_status, std::vector<base::commands::Joints> &solution,
                        KinematicsStatus &solver_status)
{
    convertPoseBetweenDifferentFrames(kdl_tree_, target_pose, kinematic_pose_);

    getKinematicJoints(kdl_kinematic_chain_, joint_status, jt_names_, current_jt_status_);

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
    getKinematicJoints(kdl_kinematic_chain_, joint_angles, jt_names_, current_jt_status_);

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

void KdlSolver::getChainSegementPose(const base::samples::Joints &joint_angles,  std::vector<KDL::Frame> &segement_pose)
{
    getKinematicJoints(kdl_kinematic_chain_, joint_angles, jt_names_, current_jt_status_);

    convertVectorToKDLArray(current_jt_status_, kdl_jtArray_);
    
    segement_pose.resize(kdl_kinematic_chain_.getNrOfSegments());
    
    fk_solverPos_->JntToCart(kdl_jtArray_, segement_pose);
    
//     for(std::size_t i = 0; i < segement_pose.size();i++)
//     {
//         std::cout<<segement_pose.at(i).M(0,0)<<"  "<<segement_pose.at(i).M(0,1)<<"  "<<segement_pose.at(i).M(0,2)<<"  "<<segement_pose.at(i).p.x()<<std::endl;
//         std::cout<<segement_pose.at(i).M(1,0)<<"  "<<segement_pose.at(i).M(1,1)<<"  "<<segement_pose.at(i).M(1,2)<<"  "<<segement_pose.at(i).p.y()<<std::endl;
//         std::cout<<segement_pose.at(i).M(2,0)<<"  "<<segement_pose.at(i).M(2,1)<<"  "<<segement_pose.at(i).M(2,2)<<"  "<<segement_pose.at(i).p.z()<<std::endl<<std::endl;
//         double qx,qy,qz,qw;
//     segement_pose.at(i).M.GetQuaternion(qx,qy,qz,qw);
//     std::cout<<"quat = "<<qx<<"  "<<qy<<"  "<<qz<<"  "<<qw<<std::endl;
//     std::cout<<std::endl;
// 
//     }

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
