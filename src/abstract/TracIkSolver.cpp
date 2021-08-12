#include "kinematics_library/HandleKinematicConfig.hpp"

namespace kinematics_library
{

TracIkSolver::TracIkSolver (const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain ): 
                            kdl_kinematic_chain_(kdl_kinematic_chain)
{
    kdl_tree_  = kdl_tree;
    kdl_chain_ = kdl_chain;
}

TracIkSolver::~TracIkSolver()
{
    trac_ik_solver_.reset();
    if ( fk_kdlsolver_pos_ ) 
    {
        delete fk_kdlsolver_pos_;
        fk_kdlsolver_pos_ = NULL;        
    }
}

bool TracIkSolver::loadKinematicConfig( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status)
{
    // assign the config
    YAML::Node input_config;
    // check whether the config could be loaded or not.
    if(!handle_kinematic_config::loadConfigFile(kinematics_config.solver_config_abs_path, kinematics_config.solver_config_filename, input_config))
    {
        LOG_ERROR("[TracIkSolver]: Unable to load kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(), 
                    kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::NO_CONFIG_FILE;
        return false;
    }
     
    const YAML::Node& trac_ik_config_node = input_config["trac_ik_config"];
    if(!handle_kinematic_config::getTracIkConfig(trac_ik_config_node, trac_ik_config_))
    {
        LOG_ERROR("[TracIkSolver]: Unable to read kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(), 
                    kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::CONFIG_READ_ERROR;
        return false;
    }
	
	TRAC_IK::SolveType solverType = TRAC_IK::Speed;
	switch(trac_ik_config_.solver_type)
	{
		case SPEED:
			solverType = TRAC_IK::Speed;
			break;
		case DISTANCE:
			solverType = TRAC_IK::Distance;
			break;
		case MANIP1:
			solverType = TRAC_IK::Manip1;
			break;
		case MANIP2:
			solverType = TRAC_IK::Manip2;
			break;
		default:
			LOG_WARN("Undefined TRAC_IK Solver Type configured");
            kinematics_status.statuscode = KinematicsStatus::CONFIG_READ_ERROR;
            return false;
	}
	assert(trac_ik_config_.joints_err_weight.size() == kdl_chain_.getNrOfJoints());		
	KDL::JntArray qerr_wt(trac_ik_config_.joints_err_weight.size());
    
    for(uint i = 0; i < qerr_wt.data.size(); ++i)
    {
        qerr_wt(i) = trac_ik_config_.joints_err_weight.at(i);
    }
    
    trac_ik_solver_ = std::make_shared<TRAC_IK::TRAC_IK> ( kinematics_config.base_name, kinematics_config.tip_name, qerr_wt, kinematics_config.urdf_file,
                                                           trac_ik_config_.timeout_sec, trac_ik_config_.eps, solverType);
    fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_kinematic_chain_ );

    assignVariables ( kinematics_config, kdl_chain_ );
    kdl_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    kdl_ik_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    
    bounds.vel = KDL::Vector(trac_ik_config_.tolerances[0], trac_ik_config_.tolerances[1], trac_ik_config_.tolerances[2]);
    bounds.rot = KDL::Vector(trac_ik_config_.tolerances[3], trac_ik_config_.tolerances[4], trac_ik_config_.tolerances[5]);

    kinematics_status.statuscode = KinematicsStatus::SUCCESS;
    return true;
}

bool TracIkSolver::solveIK (const base::samples::RigidBodyState &target_pose, 
                            const base::samples::Joints &joint_status,
                            std::vector<base::commands::Joints> &solution,
                            KinematicsStatus &solver_status )
{
    if(!convertPoseBetweenDifferentFrames(kdl_tree_, joint_status, target_pose, kinematic_pose_))
    {
        solver_status.statuscode = KinematicsStatus::KDL_CHAIN_FAILED;
        return false;
    }

    getKinematicJoints ( kdl_chain_, joint_status, jt_names_, current_jt_status_ );

    convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );
    rbsToKdl ( kinematic_pose_, kdl_frame_ );

    int res = trac_ik_solver_->CartToJnt ( kdl_jt_array_, kdl_frame_, kdl_ik_jt_array_ , bounds);
    
    std::vector<KDL::JntArray> kdl_ik_jt_array_list;
    if (!trac_ik_solver_->getSolutions(kdl_ik_jt_array_list))
    {
        LOG_WARN("[solveIK]: Cannot able to get the solution");
        solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
        return false;
    }
    
    solution.resize(kdl_ik_jt_array_list.size());
    for(unsigned i = 0; i < kdl_ik_jt_array_list.size(); ++i)
    {
        convertKDLArrayToBaseJoints(kdl_ik_jt_array_list.at(i), solution.at(i));
        solution.at(i).names = jt_names_;
        solution.at(i).time = target_pose.time;
    }
    
    if ( res >= 0 ) 
    {
        solver_status.statuscode = KinematicsStatus::IK_FOUND;
        return true;
    }
    else if ( res == -3 ) 
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

bool TracIkSolver::solveFK (const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status )
{
    getKinematicJoints ( kdl_chain_, joint_angles, jt_names_, current_jt_status_ );

    convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );

    if ( fk_kdlsolver_pos_->JntToCart ( kdl_jt_array_, kdl_frame_ ) >= 0 ) 
    {
        kdlToRbs ( kdl_frame_, fk_pose );
        solver_status.statuscode = KinematicsStatus::FK_FOUND;
        fk_pose.sourceFrame = kinematic_pose_.sourceFrame;
        fk_pose.targetFrame = kinematic_pose_.targetFrame;
        return true;
    }

    solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
    return false;
}
}
