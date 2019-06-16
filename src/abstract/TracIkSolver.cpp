#include <abstract/TracIkSolver.hpp>

namespace kinematics_library
{

TracIkSolver::TracIkSolver ( const KinematicsConfig &kinematics_config,const KDL::Tree &kdl_tree, 
                             const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain )
{

    kdl_tree_       = kdl_tree;
    kdl_chain_      = kdl_chain;
	
	TRAC_IK::SolveType solverType = TRAC_IK::Speed;
	switch(kinematics_config.tracIKSolverType)
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
	}
	assert(kinematics_config.joints_weight.size() == kdl_chain_.getNrOfJoints());		
	KDL::JntArray qerr_wt(kinematics_config.joints_weight.size());
    
    for(uint i = 0; i < qerr_wt.data.size(); ++i)
    {
        qerr_wt(i) = kinematics_config.joints_weight.at(i);
    }
    
    trac_ik_solver_ = std::make_shared<TRAC_IK::TRAC_IK> ( kinematics_config.base_name, kinematics_config.tip_name, qerr_wt, kinematics_config.urdf_file,
                                                           kinematics_config.timeout_sec, kinematics_config.eps, solverType);
    fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_kinematic_chain );

    assignVariables ( kinematics_config, kdl_chain_ );
    kdl_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    kdl_ik_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    
    bounds.vel = KDL::Vector(kinematics_config.tolerances[0], kinematics_config.tolerances[1], kinematics_config.tolerances[2]);
    bounds.rot = KDL::Vector(kinematics_config.tolerances[3], kinematics_config.tolerances[4], kinematics_config.tolerances[5]);
    
    
}

TracIkSolver::~TracIkSolver()
{
    trac_ik_solver_.reset();
    if ( fk_kdlsolver_pos_ ) 
    {
        fk_kdlsolver_pos_ = NULL;
        delete fk_kdlsolver_pos_;
    }
}

bool TracIkSolver::solveIK (const base::samples::RigidBodyState target_pose, 
                            const base::samples::Joints &joint_status,
                            std::vector<base::commands::Joints> &solution,
                            KinematicsStatus &solver_status )
{
    convertPoseBetweenDifferentFrames ( kdl_tree_, target_pose, kinematic_pose_ );

    getKinematicJoints ( kdl_chain_, joint_status, jt_names_, current_jt_status_ );

    convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );
    rbsToKdl ( kinematic_pose_, kdl_frame_ );

    int res = trac_ik_solver_->CartToJnt ( kdl_jt_array_, kdl_frame_, kdl_ik_jt_array_ , bounds);
    
    std::vector<KDL::JntArray> kdl_ik_jt_array_list;
    bool valid_soln = trac_ik_solver_->getSolutions(kdl_ik_jt_array_list);
    
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
        kdlToRbs ( kdl_frame_, kinematic_pose_ );
        solver_status.statuscode = KinematicsStatus::FK_FOUND;
        convertPoseBetweenDifferentFrames ( kdl_tree_, kinematic_pose_, fk_pose );
        return true;
    }

    solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
    return false;
}
}
