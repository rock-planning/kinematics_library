#include <abstract/TracIkSolver.hpp>

namespace kinematics_library
{
    
    TracIkSolver::TracIkSolver(const std::string base_link, const std::string tip_link, const std::string urdf_file_path, const KDL::Tree &kdl_tree,
							   const KDL::Chain &kdl_chain, const unsigned int max_iter, const double eps): 
							   base_link_(base_link), tip_link_(tip_link), urdf_file_path_(urdf_file_path), max_iter_(max_iter) , eps_(eps)
    {
		kdl_tree_ 			= kdl_tree;
		kdl_chain_			= kdl_chain;
        trac_ik_solver_		= std::make_shared<TRAC_IK::TRAC_IK> (base_link_, tip_link_, urdf_file_path_, max_iter_, eps_);
		fk_kdlsolver_pos_   = new KDL::ChainFkSolverPos_recursive(kdl_chain_);
		resize_variables(kdl_chain_);
    }

    TracIkSolver::~TracIkSolver()
    {
		trac_ik_solver_->reset();
		if (fk_kdlsolver_pos_)
		{
			fk_kdlsolver_pos_ = NULL;
			delete fk_kdlsolver_pos_;
		}
    }

    bool TracIkSolver::solveIK(	const base::samples::RigidBodyState target_pose,
								const base::samples::Joints &joint_status,
								base::commands::Joints &solution,
								KinematicsStatus &solver_status)
    {
		convertPoseBetweenDifferentFrames(kdl_tree_, target_pose, kinematic_pose_);
		
		getKinematicJoints(joint_status, kdl_chain_, jt_names_, current_jt_status_);

        convertVectorToKDLArray(current_jt_status_, kdl_jt_array_);
		rbsToKdl(kinematic_pose_, kdl_frame_);                

        int res = trac_ik_solver_->CartToJnt(kdl_jt_array_, kdl_frame_, kdl_ik_jt_array_);
        if( res >= 0)
        {
			convertKDLArrayToBaseJoints(kdl_ik_jt_array_, solution);
			solution.names = jt_names_;
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

    bool TracIkSolver::solveFK(	const base::samples::Joints &joint_angles,
								base::samples::RigidBodyState &fk_pose,
								KinematicsStatus &solver_status)
    {
        convertVectorToKDLArray(joint_angles, kdl_jt_array_);

        if(fk_kdlsolver_pos_->JntToCart(kdl_jt_array_, kdl_frame_) >= 0)
        {
			kdlToRbs(kdl_frame_, kinematic_pose_); 
			solver_status.statuscode = KinematicsStatus::FK_FOUND;
			convertPoseBetweenDifferentFrames(kdl_tree_, kinematic_pose_, fk_pose);
            return true;
        }
        
        solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
        return false;        
    }   
}
