#include "RobotKinematics.hpp"
#include <stdio.h>
#include <stdlib.h>

namespace kinematics_library
{

    RobotKinematics::RobotKinematics(KinematicsConfig _kinematicsconfig, KDL::Tree _kinematicsKDLTree) :
        kinematics_config_(_kinematicsconfig),kdl_tree_(_kinematicsKDLTree)
    {}
    
    bool RobotKinematics::initialise()
    {
        std::map<std::string,KDL::TreeElement>::const_iterator root = kdl_tree_.getRootSegment();
        root_name_ = root->first;

        base_name_ = kinematics_config_.base_name;
        tip_name_ = kinematics_config_.tip_name;

        if(!kdl_tree_.getChain(base_name_, tip_name_, kdl_chain_))
	{
		LOG_FATAL("[RobotKinematics]: Could not initiailise KDL chain !!!!!!!");            
		exit(1);
		return false;
	}
        else
	    LOG_INFO("[RobotKinematics]: KDL chain initialised with size %d",kdl_chain_.segments.size());            

        for(std::size_t i=0; i<kdl_chain_.segments.size(); i++ )
        {
            if(! (kdl_chain_.getSegment(i).getJoint().getType()==KDL::Joint::None) )
            {
                rev_jt_kdlchain_.addSegment(kdl_chain_.getSegment(i));
            }
        }
			
	transform_pose_.Identity();
	transformFrame( kdl_tree_, root_name_, base_name_, transform_pose_);
	
		
        current_jt_status_.resize(rev_jt_kdlchain_.segments.size(),0.0);
        jt_names_.resize(rev_jt_kdlchain_.segments.size(),"");
        ik_solution_.resize(rev_jt_kdlchain_.segments.size(),0.0);
		

        //pack the joint limit as std::pair
        joints_limits_.clear();
        for(std::size_t jn = 0; jn < kinematics_config_.number_of_joints; jn++)
        {
            joints_limits_.push_back(std::make_pair(kinematics_config_.min_joints_limits.at(jn), kinematics_config_.max_joints_limits.at(jn)));
        }
	
        switch(kinematics_config_.kinematic_solver)
        {
            case IKFAST:
            {
                kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new IkFastSolver( kinematics_config_.number_of_joints, kinematics_config_.joints_weight, 
                                                                                            joints_limits_, ComputeIk,ComputeFk) );
                break;
            }
            case CUSTOM_SOLVER:
            {
                //kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new ArmSolver(kinematics_config_.mKinematicConstraints));
                break;
            }
            case KDL:
            {
                kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new KdlSolver(kinematics_config_.number_of_joints, joints_limits_, rev_jt_kdlchain_));
                break;
            }
            default:
            {
                LOG_ERROR("[RobotKinematics]: This  kinematicSolver is not available");
                throw new std::runtime_error("This kinematicSolver is not available");
		return false;
            }
        }
        LOG_DEBUG("[RobotKinematics]: Initiailising finished");  
        return true;
    }

    RobotKinematics::~RobotKinematics()
    {
        kinematics_solver_.reset();
    }

    void RobotKinematics::transformFrame( const KDL::Tree &kdl_tree, const std::string &base_link, const std::string &tip_link, KDL::Frame &pose)
    {
	KDL::Chain new_chain;

	if(!kdl_tree.getChain(base_link , tip_link , new_chain))
	{
	    LOG_FATAL("[RobotKinematics]: Could not initiailise KDL transformation chain !!!!!!!");
	    exit(1);
	}
	else
	LOG_INFO("[RobotKinematics]: KDL transformation chain initilised");

	for(std::size_t i=0; i<new_chain.segments.size(); i++ )
	{
		pose = pose * new_chain.getSegment(i).getFrameToTip();
	}	
    }

    void RobotKinematics::solveFK( const base::samples::Joints &joint_angles,
                                    base::samples::RigidBodyState &result_pose,
                                    KinematicsStatus &solver_status)
    {
        for(unsigned int i = 0; i < rev_jt_kdlchain_.segments.size(); i++)
            current_jt_status_.at(i) = joint_angles[rev_jt_kdlchain_.getSegment(i).getJoint().getName()].position;

        kinematics_solver_->getFK(base_name_, tip_name_, current_jt_status_,
                                result_pose.position, result_pose.orientation, solver_status);

		LOG_DEBUG("[RobotKinematics]: Rootname = %s Basename = %s", root_name_.c_str(), base_name_.c_str());
		
	    if(root_name_ != base_name_)
		{
			KDL::Frame calculated_frame, new_frame;
		    calculated_frame.p.data[0] = result_pose.position(0);
		    calculated_frame.p.data[1] = result_pose.position(1);
		    calculated_frame.p.data[2] = result_pose.position(2);

		    calculated_frame.M = KDL::Rotation::Quaternion(	result_pose.orientation.x(), result_pose.orientation.y(),
		                                            		result_pose.orientation.z(), result_pose.orientation.w() );

			new_frame = transform_pose_ * calculated_frame;

			result_pose.position(0) = new_frame.p.data[0];
		    result_pose.position(1) = new_frame.p.data[1];
		    result_pose.position(2) = new_frame.p.data[2];

		    new_frame.M.GetQuaternion(	result_pose.orientation.x(), result_pose.orientation.y(), 
									result_pose.orientation.z(), result_pose.orientation.w());
		}

        result_pose.sourceFrame = base_name_;
        result_pose.targetFrame = tip_name_;

    }

    bool RobotKinematics::solveIK(const base::Vector3d &target_position,
                                   const base::Quaterniond &target_orientation,
                                   const base::samples::Joints &joint_status,
                                   base::commands::Joints &solution,
                                   KinematicsStatus &solver_status)
    {

		LOG_WARN("[RobotKinematics]: IK function called ");
		LOG_WARN("[RobotKinematics]: Position:/n X: %f Y: %f Z: %f",
					target_position(0), target_position(1), target_position(2));
		LOG_WARN("[RobotKinematics]: Orientation:/n X: %f Y: %f Z: %f W: %f",
        			target_orientation.x(), target_orientation.y(), target_orientation.z(), target_orientation.w());

        for(unsigned int i = 0; i < rev_jt_kdlchain_.segments.size(); i++)
        {
            current_jt_status_.at(i) = joint_status[rev_jt_kdlchain_.getSegment(i).getJoint().getName()].position;
            jt_names_.at(i)          = rev_jt_kdlchain_.getSegment(i).getJoint().getName();
        }


        if(kinematics_solver_->getIK(base_name_, target_position, target_orientation,
                                   current_jt_status_, ik_solution_, solver_status))
        {            
            solution = base::samples::Joints::Positions(ik_solution_,jt_names_);
	    for (std::size_t i = 0; i != solution.elements.size(); ++i)
	    {
	    	solution[i].speed = 0.0;
	    	solution[i].effort = 0.0;
	     }
	    
            return true;
        }
        else
            return false;
    }

    bool RobotKinematics::solveIKRelatively(  const base::samples::Joints &joint_angles,
                                                    const base::samples::RigidBodyState &relative_pose,
                                                    base::commands::Joints &solution,
                                                    KinematicsStatus &solver_status)

    {
		LOG_WARN("[RobotKinematics]: solveIKRelatively function called ");

        // calculate the forward kinematics for the current joint angles
		base::samples::RigidBodyState fk;
    	this->solveFK( joint_angles,  fk, solver_status);
        // get the forward kinematics in a homogeneous matrix form
        Eigen::Matrix4d fk_matrix;
        getHomogeneousMatrix(fk.position, fk.orientation, fk_matrix);

        base::Vector3d eulerrot;
        quaternionToEuler(fk.orientation, eulerrot);
        std::cout<<" FK euler = "<<eulerrot.x()<<"  "<<eulerrot.y()<<"  "<<eulerrot.z()<<std::endl;
        // get the relative pose in a homogeneous matrix form
        Eigen::Matrix4d relative_pose_matrix;
        getHomogeneousMatrix(relative_pose.position, relative_pose.orientation, relative_pose_matrix);

        eulerrot = base::Position::Zero();
        quaternionToEuler(relative_pose.orientation, eulerrot);
        std::cout<<" relative euler = "<<eulerrot.x()<<"  "<<eulerrot.y()<<"  "<<eulerrot.z()<<std::endl;
        std::cout<<" relative Quaternion = "<<relative_pose.orientation.x()<<"  "<<relative_pose.orientation.y()<<"  "<<relative_pose.orientation.z()<<"  "<<
            relative_pose.orientation.w()<<std::endl;
        Eigen::Matrix4d target_matrix = fk_matrix * relative_pose_matrix;
    
        base::Vector3d target_position;
        base::Quaterniond target_orientation;
        getPositionRotation(target_matrix, target_position, target_orientation);

        eulerrot = base::Position::Zero();
        quaternionToEuler(target_orientation, eulerrot);
        std::cout<<" target euler = "<<eulerrot.x()<<"  "<<eulerrot.y()<<"  "<<eulerrot.z()<<std::endl;
        //solve inverse kinematics
        return solveIK( target_position, target_orientation, joint_angles, solution, solver_status);

	}

}
