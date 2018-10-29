#include "KinematicsFactory.hpp"
#include <stdio.h>
#include <stdlib.h>

namespace kinematics_library
{

KinematicsFactory::KinematicsFactory()
{}

KinematicsFactory::~KinematicsFactory()
{}

AbstractKinematicPtr KinematicsFactory::getKinematicsSolver(const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status)
{
	AbstractKinematicPtr kinematic_solver = NULL;	
	
	if(!initialise(kinematics_config, kinematics_status))
		return NULL;
	
	switch(kinematics_config.kinematic_solver)
	{
		case IKFAST:
		{
			LOG_INFO_S<<"[KinematicsFactory]: IKFAST solver is selected";
			
			if(!getIKFASTFunctionPtr(kinematics_config.ikfast_lib, kinematics_status, computeFkFn, computeIkFn))
				return NULL;
			
			kinematic_solver = std::shared_ptr<IkFastSolver> (new IkFastSolver( kinematics_config.number_of_joints, kinematics_config.joints_weight, 
																	joints_limits_, kdl_tree_, rev_jt_kdlchain_, computeFkFn, computeIkFn) );
			break;
		}
		case KDL:
		{
			LOG_INFO_S<<"[KinematicsFactory]: KDL solver is selected";
			kinematic_solver = std::shared_ptr<KdlSolver> (new KdlSolver(kinematics_config.number_of_joints, joints_limits_, kdl_tree_, rev_jt_kdlchain_,
																						kinematics_config.max_iteration, kinematics_config.eps));
			break;
		}
		case TRACIK:
		{
			LOG_INFO_S<<"[KinematicsFactory]: TRACIK solver is selected";
			#if(TRACK_IK_LIB_FOUND)
				kinematic_solver = std::shared_ptr<TracIkSolver> (new TracIkSolver(kinematics_config.base_name, kinematics_config.tip_name, kinematics_config.urdf_file,
																							 kdl_tree_, rev_jt_kdlchain_, kinematics_config.max_iteration, kinematics_config.eps));
			#else
				LOG_FATAL_S << "[KinematicsFactory]: TRACIK is not installed. Please select an another solver !";
				return NULL;
			#endif
			
			break;
		}
		default:
		{
			kinematics_status.statuscode = KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND;
			LOG_ERROR("[KinematicsFactory]: This  kinematicSolver is not available");
			throw new std::runtime_error("This kinematicSolver is not available");
			return NULL;
		}
	}	
	return kinematic_solver;
}


    
bool KinematicsFactory::initialise(const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status)
{
	LOG_DEBUG("[KinematicsFactory]: Initialising kinematic factory d%s", kinematics_config.urdf_file.c_str());

	if (!kdl_parser::treeFromFile(kinematics_config.urdf_file, kdl_tree_))
	{
		kinematics_status.statuscode = KinematicsStatus::KDL_TREE_FAILED;
		LOG_FATAL_S<<"[RobotKinematics]: Error while initialzing KDL tree";
	}

	if(!kdl_tree_.getChain(kinematics_config.base_name, kinematics_config.tip_name, kdl_chain_))
	{
		kinematics_status.statuscode = KinematicsStatus::KDL_CHAIN_FAILED;    
		LOG_FATAL("[RobotKinematics]: Could not initiailise KDL chain !");            	
		return false;
	}
	else
		LOG_INFO("[RobotKinematics]: KDL chain initialised with size %d",kdl_chain_.segments.size());            

	for(std::size_t i=0; i<kdl_chain_.segments.size(); i++ )
	{
		if(! (kdl_chain_.getSegment(i).getJoint().getType()==KDL::Joint::None) )			
			rev_jt_kdlchain_.addSegment(kdl_chain_.getSegment(i));			
	}

	if(!initiailiseURDF(kinematics_config.urdf_file))
	{
		kinematics_status.statuscode = KinematicsStatus::URDF_FAILED;
		return true;
	}
	
	//pack the joint limit as std::pair
	joints_limits_.clear();
	for(std::size_t jn = 0; jn < kinematics_config.number_of_joints; jn++)
	{            
		joints_limits_.push_back(std::make_pair(urdf_model_->getJoint(rev_jt_kdlchain_.getSegment(jn).getJoint().getName())->limits->lower, 
								urdf_model_->getJoint(rev_jt_kdlchain_.getSegment(jn).getJoint().getName())->limits->upper));
	}      

	
	LOG_DEBUG("[KinematicsFactory]: Initiailising finished");  
	return true;
}

bool KinematicsFactory::initiailiseURDF(std::string urdf_file)
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
			LOG_ERROR("[KinematicsFactory] Error while getting urdf model. urdf_model is empty");
			return false;
		}
	}
	else
	{
		LOG_ERROR("[KinematicsFactory] Cannot open urdf file.");
		return false;
	}
	return true;
}  

bool KinematicsFactory::getIKFASTFunctionPtr(const std::string ikfast_lib, KinematicsStatus &kinematics_status, void (*ComputeFkFn)(const IkReal* j, IkReal* eetrans, IkReal* eerot),
											 bool (*ComputeIkFn)(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions))
{

    void *handle;
    char *error;

    handle = dlopen (ikfast_lib.c_str(), RTLD_LAZY);
    if (!handle)
    {
        LOG_ERROR("[KinematicsFactory]: Cannot open ikfst shared library. Error %s",dlerror());
        kinematics_status.statuscode = KinematicsStatus::IKFAST_LIB_NOT_AVAILABLE;
        return false;
    }
    
    // get the forward kinematics fuction pointer
    ComputeFkFn=(  void (*) ( const IkReal* , IkReal* , IkReal* ) )  dlsym(handle, "ComputeFk");
	
    if ((error = dlerror()) != NULL)
    {
        LOG_ERROR("[KinematicsFactory]: Cannot find ComputeFk function. Error %s",dlerror());        
        dlclose(handle);
        kinematics_status.statuscode = KinematicsStatus::IKFAST_FUNCTION_NOT_FOUND;
        return false;
    }
    
    // get the inverse kinematics fuction pointer
    ComputeIkFn = (bool (*) (const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& ) )  dlsym(handle, "ComputeIk");
	if ((error = dlerror()) != NULL)
    {
		LOG_ERROR("[KinematicsFactory]: Cannot find ComputeIk function. Error %s",dlerror());        
        dlclose(handle);
        kinematics_status.statuscode = KinematicsStatus::IKFAST_FUNCTION_NOT_FOUND;
        return false;
    }    
    return true;
}

//     void RobotKinematics::solveFK( const base::samples::Joints &joint_angles,
//                                     base::samples::RigidBodyState &result_pose,
//                                     KinematicsStatus &solver_status)
//     {
// 		for(unsigned int i = 0; i < rev_jt_kdlchain_.segments.size(); i++)
// 			current_jt_status_.at(i) = joint_angles[rev_jt_kdlchain_.getSegment(i).getJoint().getName()].position;
// 
// 		kinematics_solver_->getFK(current_jt_status_, kinematic_pose_, solver_status);	
// 
// 		convertPoseBetweenDifferentFrames(kdl_tree_, kinematic_pose_, result_pose);	
//     }
// 
//     bool RobotKinematics::solveIK(const base::samples::RigidBodyState &target_pose,                                   
//                                   const base::samples::Joints &joint_status,
//                                   base::commands::Joints &solution,
//                                   KinematicsStatus &solver_status)
//     {
// 
// 		LOG_DEBUG_S<<"[RobotKinematics]: IK function called for target pose ";
// 		LOG_DEBUG("[RobotKinematics]: Position:/n X: %f Y: %f Z: %f", target_pose.position(0), target_pose.position(1), target_pose.position(2));		
// 		LOG_DEBUG("[RobotKinematics]: Orientation:/n X: %f Y: %f Z: %f W: %f",
// 		target_pose.orientation.x(), target_pose.orientation.y(), target_pose.orientation.z(), target_pose.orientation.w());
// 
// 		convertPoseBetweenDifferentFrames(kdl_tree_, target_pose, kinematic_pose_);	
// 
// 		LOG_DEBUG_S<<"[RobotKinematics]: Target pose after frame transformation ";
// 		LOG_DEBUG("[RobotKinematics]: Position:/n X: %f Y: %f Z: %f", kinematic_pose_.position(0), kinematic_pose_.position(1), kinematic_pose_.position(2));		
// 		LOG_DEBUG("[RobotKinematics]: Orientation:/n X: %f Y: %f Z: %f W: %f",
// 		kinematic_pose_.orientation.x(), kinematic_pose_.orientation.y(), kinematic_pose_.orientation.z(), kinematic_pose_.orientation.w());
// 
// 
//         for(unsigned int i = 0; i < rev_jt_kdlchain_.segments.size(); i++)
//         {
//             current_jt_status_.at(i) = joint_status[rev_jt_kdlchain_.getSegment(i).getJoint().getName()].position;
//             jt_names_.at(i)          = rev_jt_kdlchain_.getSegment(i).getJoint().getName();
//         }	
// 
//         if(kinematics_solver_->getIK(kinematic_pose_, current_jt_status_, ik_solution_, solver_status))
//         {
// 			LOG_INFO_S<<"[RobotKinematics]: IK Found";
//             solution = base::samples::Joints::Positions(ik_solution_,jt_names_);
// 			for (std::size_t i = 0; i != solution.elements.size(); ++i)
// 			{
// 				solution[i].speed = 0.0;
// 				solution[i].effort = 0.0;
// 			}
// 	    
//             return true;
//         }
//         else
//             return false;
//     }
// 
//     bool RobotKinematics::solveIKRelatively(const base::samples::Joints &joint_angles,
// 											const base::samples::RigidBodyState &relative_pose,
// 											base::commands::Joints &solution,
// 											KinematicsStatus &solver_status)
// 
//     {
// 		LOG_WARN("[RobotKinematics]: solveIKRelatively function called ");
// 
//         // calculate the forward kinematics for the current joint angles
// 		base::samples::RigidBodyState fk;
//     	this->solveFK( joint_angles,  fk, solver_status);
//         // get the forward kinematics in a homogeneous matrix form
//         Eigen::Matrix4d fk_matrix;
//         getHomogeneousMatrix(fk.position, fk.orientation, fk_matrix);
// 
//         base::Vector3d eulerrot;
//         quaternionToEuler(fk.orientation, eulerrot);
//         std::cout<<" FK euler = "<<eulerrot.x()<<"  "<<eulerrot.y()<<"  "<<eulerrot.z()<<std::endl;
//         // get the relative pose in a homogeneous matrix form
//         Eigen::Matrix4d relative_pose_matrix;
//         getHomogeneousMatrix(relative_pose.position, relative_pose.orientation, relative_pose_matrix);
// 
//         eulerrot = base::Position::Zero();
//         quaternionToEuler(relative_pose.orientation, eulerrot);
//         std::cout<<" relative euler = "<<eulerrot.x()<<"  "<<eulerrot.y()<<"  "<<eulerrot.z()<<std::endl;
//         std::cout<<" relative Quaternion = "<<relative_pose.orientation.x()<<"  "<<relative_pose.orientation.y()<<"  "<<relative_pose.orientation.z()<<"  "<<
//             relative_pose.orientation.w()<<std::endl;
//         Eigen::Matrix4d target_matrix = fk_matrix * relative_pose_matrix;
//     
//         base::Vector3d target_position;
//         base::Quaterniond target_orientation;
//         getPositionRotation(target_matrix, target_position, target_orientation);
// 
//         eulerrot = base::Position::Zero();
//         quaternionToEuler(target_orientation, eulerrot);
//         std::cout<<" target euler = "<<eulerrot.x()<<"  "<<eulerrot.y()<<"  "<<eulerrot.z()<<std::endl;
// 
// 		base::samples::RigidBodyState target_pose;
// 
//         //solve inverse kinematics
//         return solveIK( target_pose, joint_angles, solution, solver_status);
// 
// 	}
// 
// }
// 
// 
}
