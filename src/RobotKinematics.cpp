#include "RobotKinematics.hpp"
#include <stdio.h>
#include <stdlib.h>

namespace kinematics_library
{

    RobotKinematics::RobotKinematics(KinematicsConfig _kinematicsconfig, KDL::Tree _kinematicsKDLTree) :
        kinematics_config_(_kinematicsconfig),kdl_tree_(_kinematicsKDLTree)
    {
        std::map<std::string,KDL::TreeElement>::const_iterator root = kdl_tree_.getRootSegment();
        root_name_ = root->first;

        base_name_ = kinematics_config_.mBaseName;
        tip_name_ = kinematics_config_.mTipName;

        if(!kdl_tree_.getChain(base_name_, tip_name_, kdl_chain_))
		{
			LOG_FATAL("[RobotKinematics]: Could not initiailise KDL chain !!!!!!!");            
			exit(1);
		}
        else
			LOG_INFO("[RobotKinematics]: KDL chain initialised");            

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
        for(std::size_t jn = 0; jn < kinematics_config_.mNumberOfJoints; jn++)
        {
            joints_limits_.push_back(std::make_pair(kinematics_config_.mMinJointsLimits.at(jn), kinematics_config_.mMaxJointsLimits.at(jn)));
        }

        switch(kinematics_config_.mKinematicSolver)
        {
            case IKFAST:
            {
                kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new IkFastSolver( kinematics_config_.mNumberOfJoints, kinematics_config_.mJointsWeight, 
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
                kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new KdlSolver(kinematics_config_.mNumberOfJoints, joints_limits_, rev_jt_kdlchain_));
                break;
            }
            default:
            {
                LOG_ERROR("[RobotKinematics]: This  kinematicSolver is not available");
                throw new std::runtime_error("This kinematicSolver is not available");
            }
        }
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

    bool RobotKinematics::solveIK( const std::vector < base::samples::RigidBodyState> &target_poses,
                                    const base::samples::Joints &joint_status,
                                    base::JointsTrajectory &solutions,
                                    KinematicsStatus &solver_status)
    {

		LOG_WARN("[RobotKinematics]: IK for vector of poses function called ");
		

        /*for(int i =0; i <target_poses.size(); i++)
        {
            std::cout<<"Target Frame" <<target_poses.at(i).targetFrame.c_str()<<std::endl;
            std::cout<<target_poses.at(i).position(0)<<" "<<target_poses.at(i).position(1)<<"  "<<target_poses.at(i).position(2)<<std::endl;
            std::cout<<target_poses.at(i).orientation.x()<<" "<<target_poses.at(i).orientation.y()<<"  "<<target_poses.at(i).orientation.z()<<" "<<target_poses.at(i).orientation.w()<<std::endl;
        }*/



        for(unsigned int i = 0; i < rev_jt_kdlchain_.segments.size(); i++)
        {
            current_jt_status_.at(i) = joint_status[rev_jt_kdlchain_.getSegment(i).getJoint().getName()].position;
            jt_names_.at(i)          = rev_jt_kdlchain_.getSegment(i).getJoint().getName();
        }

        // assiging the solution size
        solutions.resize(current_jt_status_.size(), target_poses.size());
        solutions.names = jt_names_;

        // solving the ik for the given target poses
        for (unsigned int j = 0; j < target_poses.size(); j++ )
        {
			LOG_DEBUG("[RobotKinematics]: Position:/n X: %f Y: %f Z: %f",
						target_poses.at(j).position(0), target_poses.at(j).position(1), target_poses.at(j).position(2));
			LOG_DEBUG("[RobotKinematics]: Orientation:/n X: %f Y: %f Z: %f W: %f",
		    			target_poses.at(j).orientation.x(), target_poses.at(j).orientation.y(), target_poses.at(j).orientation.z(),
						target_poses.at(j).orientation.w());
            

            if(kinematics_solver_->getIK(base_name_, target_poses.at(j).position, target_poses.at(j).orientation,
                                       current_jt_status_, ik_solution_, solver_status))
            {
                for (unsigned int jt_no = 0; jt_no < current_jt_status_.size(); jt_no++ )
                    solutions.elements.at(jt_no).at(j).position = ik_solution_.at(jt_no);

                // assign the calcualted ik solution as current jt angle for the next iteration
                current_jt_status_ = ik_solution_;
            }
            else
            {
                return false;
            }
        }

        return true;
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
