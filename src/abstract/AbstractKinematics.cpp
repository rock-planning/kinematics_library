#include <abstract/AbstractKinematics.hpp>

namespace kinematics_library
{

AbstractKinematics::AbstractKinematics()
{}

AbstractKinematics::~AbstractKinematics()
{}

void AbstractKinematics::resize_variables(const KDL::Chain &kdl_chain)
{
	current_jt_status_.resize(kdl_chain.segments.size(),0.0);
	jt_names_.resize(kdl_chain.segments.size(),"");
	ik_solution_.resize(kdl_chain.segments.size(),0.0);
}

bool AbstractKinematics::solveIKRelatively(const base::samples::Joints &joint_angles,
											const base::samples::RigidBodyState &relative_pose,
											base::commands::Joints &solution,
											KinematicsStatus &solver_status)

{
	LOG_DEBUG("[AbstractKinematics]: solveIKRelatively function called ");

	// calculate the forward kinematics for the current joint angles
	base::samples::RigidBodyState fk;
	solveFK( joint_angles,  fk, solver_status);
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
