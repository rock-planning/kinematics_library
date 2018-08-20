#include "RobotKinematics.hpp"
#include <stdio.h>
#include <stdlib.h>

namespace kinematics_library
{

    RobotKinematics::RobotKinematics(KinematicsConfig _kinematicsconfig) :
        kinematics_config_(_kinematicsconfig)
    {	
    }
    
    bool RobotKinematics::initialise(KinematicsStatus &kinematics_status)
    {

	if (!kdl_parser::treeFromFile(kinematics_config_.urdf_file, kdl_tree_))
	{
	    kinematics_status.statuscode = KinematicsStatus::KDL_CHAIN_FAILED;
	    LOG_FATAL_S<<"[KinematicLibraryTask]: Error while initialzing KDL tree";
	}

        kin_base_name_ = kinematics_config_.base_name;
        kin_tip_name_ = kinematics_config_.tip_name;
	
	kinematic_pose_.sourceFrame = kin_base_name_;
	kinematic_pose_.targetFrame = kin_tip_name_;

        if(!kdl_tree_.getChain(kin_base_name_, kin_tip_name_, kdl_chain_))
	{
	    kinematics_status.statuscode = KinematicsStatus::KDL_INITIALISATION_FAILED;    
	    LOG_FATAL("[RobotKinematics]: Could not initiailise KDL chain !!!!!!!");            	
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
        
        if(!initiailiseURDF(kinematics_config_.urdf_file))
	{
	    kinematics_status.statuscode = KinematicsStatus::URDF_FAILED;
	    return true;
	}			

	//transformFrame( kdl_tree_, root_name_, base_name_, transform_pose_);

        current_jt_status_.resize(rev_jt_kdlchain_.segments.size(),0.0);
        jt_names_.resize(rev_jt_kdlchain_.segments.size(),"");
        ik_solution_.resize(rev_jt_kdlchain_.segments.size(),0.0);
		

        //pack the joint limit as std::pair
        joints_limits_.clear();
        for(std::size_t jn = 0; jn < kinematics_config_.number_of_joints; jn++)
        {            
	    joints_limits_.push_back(std::make_pair(urdf_model_->getJoint(rev_jt_kdlchain_.getSegment(jn).getJoint().getName())->limits->lower, 
						    urdf_model_->getJoint(rev_jt_kdlchain_.getSegment(jn).getJoint().getName())->limits->upper));
        }      
	
        switch(kinematics_config_.kinematic_solver)
        {
            case IKFAST:
            {
		LOG_INFO_S<<"[RobotKinematics]: IKFAST solver is selected";
                kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new IkFastSolver( kinematics_config_.number_of_joints, kinematics_config_.joints_weight, 
                                                                                            joints_limits_, ComputeIk,ComputeFk) );
                break;
            }
            case CUSTOM_SOLVER:
            {
		LOG_INFO_S<<"[RobotKinematics]: CUSTOM solver is selected";
                //kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new ArmSolver(kinematics_config_.mKinematicConstraints));
                break;
            }
            case KDL:
            {
		LOG_INFO_S<<"[RobotKinematics]: KDL solver is selected";
                kinematics_solver_ = boost::shared_ptr<AbstractKinematics> (new KdlSolver(kinematics_config_.number_of_joints, joints_limits_, rev_jt_kdlchain_));
                break;
            }
            default:
            {
		kinematics_status.statuscode = KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND;
                LOG_ERROR("[RobotKinematics]: This  kinematicSolver is not available");
                throw new std::runtime_error("This kinematicSolver is not available");
		return false;
            }
        }
        LOG_DEBUG("[RobotKinematics]: Initiailising finished");  
        return true;
    }
    
    bool RobotKinematics::initiailiseURDF(std::string urdf_file)
    {
    	std::string xml_string;
	std::fstream xml_file(urdf_file.c_str(), std::fstream::in);
	
	if (xml_file.is_open())
	{
	    while ( xml_file.good() )
	    {
	        std::string line;
	        std::getline( xml_file, line);
	        xml_string += (line + "\n");
	    }
		xml_file.close();
		urdf_model_ = urdf::parseURDF(xml_string);
		if(urdf_model_.get() == NULL)
		{
		    LOG_ERROR("[RobotKinematics] Error while getting urdf model. urdf_model is empty");
		    return false;
		}
	}
	else
	{
	    LOG_ERROR("[RobotKinematics] Cannot open urdf file.");
		return false;
	}
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
	

        kinematics_solver_->getFK(kin_base_name_, kin_tip_name_, current_jt_status_,
                                kinematic_pose_.position, kinematic_pose_.orientation, solver_status);	
	
	convertPoseBetweenDifferentFrames(kdl_tree_, kinematic_pose_, result_pose);	
	
    }

    bool RobotKinematics::solveIK(const base::samples::RigidBodyState &target_pose,                                   
                                  const base::samples::Joints &joint_status,
                                  base::commands::Joints &solution,
                                  KinematicsStatus &solver_status)
    {

	LOG_DEBUG_S<<"[RobotKinematics]: IK function called for target pose ";
	LOG_DEBUG("[RobotKinematics]: Position:/n X: %f Y: %f Z: %f", target_pose.position(0), target_pose.position(1), target_pose.position(2));		
	LOG_DEBUG("[RobotKinematics]: Orientation:/n X: %f Y: %f Z: %f W: %f",
		target_pose.orientation.x(), target_pose.orientation.y(), target_pose.orientation.z(), target_pose.orientation.w());
	
	convertPoseBetweenDifferentFrames(kdl_tree_, target_pose, kinematic_pose_);	

	LOG_DEBUG_S<<"[RobotKinematics]: Target pose after frame transformation ";
	LOG_DEBUG("[RobotKinematics]: Position:/n X: %f Y: %f Z: %f", kinematic_pose_.position(0), kinematic_pose_.position(1), kinematic_pose_.position(2));		
	LOG_DEBUG("[RobotKinematics]: Orientation:/n X: %f Y: %f Z: %f W: %f",
		kinematic_pose_.orientation.x(), kinematic_pose_.orientation.y(), kinematic_pose_.orientation.z(), kinematic_pose_.orientation.w());
	

        for(unsigned int i = 0; i < rev_jt_kdlchain_.segments.size(); i++)
        {
            current_jt_status_.at(i) = joint_status[rev_jt_kdlchain_.getSegment(i).getJoint().getName()].position;
            jt_names_.at(i)          = rev_jt_kdlchain_.getSegment(i).getJoint().getName();
        }	

        if(kinematics_solver_->getIK(kin_base_name_, kinematic_pose_.position, kinematic_pose_.orientation,
                                   current_jt_status_, ik_solution_, solver_status))
        {
	    LOG_INFO_S<<"[RobotKinematics]: IK Found";
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
	
	base::samples::RigidBodyState target_pose;
	
        //solve inverse kinematics
        return solveIK( target_pose, joint_angles, solution, solver_status);

	}

}
