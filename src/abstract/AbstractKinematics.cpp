#include <abstract/AbstractKinematics.hpp>

namespace kinematics_library
{

AbstractKinematics::AbstractKinematics()
{}

AbstractKinematics::~AbstractKinematics()
{}

void AbstractKinematics::assign_variables(const KinematicsConfig &kinematics_config, const KDL::Chain &kdl_chain)
{
    kinematic_pose_.sourceFrame = kinematics_config.base_name;
    kinematic_pose_.targetFrame = kinematics_config.tip_name;    
    position_tolerance_in_m_    = kinematics_config.linear_movement_config.position_tolerance_in_m;
    interpolation_velocity_     = kinematics_config.linear_movement_config.interpolation_velocity;
    sampling_time_              = kinematics_config.linear_movement_config.sampling_time;
    
    current_jt_status_.resize(kdl_chain.getNrOfJoints(),0.0);
    jt_names_.resize(kdl_chain.getNrOfJoints(),"");
    ik_solution_.resize(kdl_chain.getNrOfJoints(),0.0);
    number_of_joints_ = kdl_chain.getNrOfJoints();
}

bool AbstractKinematics::solveIKRelatively(const base::samples::RigidBodyState &current_pose, const base::samples::Joints &joint_angles, 
                                           const base::samples::RigidBodyState &relative_pose,
                                           base::commands::Joints &solution, KinematicsStatus &solver_status)
{
    LOG_DEBUG("[AbstractKinematics]: solveIKRelatively function called ");

    base::samples::RigidBodyState target_pose;    
    target_pose = getRelativePose( current_pose, relative_pose);

    //solve inverse kinematics
    return solveIK( target_pose, joint_angles, solution, solver_status);
}

base::samples::RigidBodyState AbstractKinematics::getRelativePose( const base::samples::RigidBodyState &current_pose, 
                                                                   const base::samples::RigidBodyState &relative_pose)
{
    LOG_DEBUG("[AbstractKinematics]: getRelativePose function called ");

    // get the forward kinematics in a homogeneous matrix form
    Eigen::Matrix4d fk_matrix;
    getHomogeneousMatrix(current_pose.position, current_pose.orientation, fk_matrix);

    base::Vector3d eulerrot;
    quaternionToEuler(current_pose.orientation, eulerrot);
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
    
    base::samples::RigidBodyState target_pose;
    target_pose.sourceFrame = current_pose.sourceFrame;
    target_pose.targetFrame = relative_pose.targetFrame;
    getPositionRotation(target_matrix, target_pose.position, target_pose.orientation);

    eulerrot = base::Position::Zero();
    quaternionToEuler(relative_pose.orientation, eulerrot);
    std::cout<<" target euler = "<<eulerrot.x()<<"  "<<eulerrot.y()<<"  "<<eulerrot.z()<<std::endl;
    
    return target_pose;
}

bool AbstractKinematics::solveIKLinearly( const base::samples::RigidBodyState &current_pose, const base::samples::RigidBodyState &relative_pose, 
                                          const base::samples::Joints &joint_angles, base::commands::Joints &solution, double &remaining_distance,
                                          KinematicsStatus &solver_status)
{
    base::samples::RigidBodyState intermediate_pose;
    
    intermediate_pose.position = interpolate( current_pose.position, relative_pose.position, remaining_distance );
    
    //solve inverse kinematics
    return solveIK( intermediate_pose, joint_angles, solution, solver_status);
    
}

const Eigen::Vector3d AbstractKinematics::interpolate( const Eigen::Vector3d& current_position, const Eigen::Vector3d& target_position, 
                                                       double& remaining_distance )
{
    //if a new goal position for the WCP was set, "re-initialize" interpolator
    if( !( linear_target_position_ == target_position ) || !linear_movement_ )
    {
        linear_movement_ = true;

        //store the target_position for next cycle
        linear_target_position_ = target_position;
        linear_start_position_  = current_position;

        //reset scaling factor
        linear_eta_ = 0;

        //get distance and max velocity to calculate needed time
        //reset distance_to_target
        linear_distance_ =  (linear_target_position_ - linear_start_position_).norm();
        

        //get total time in seconds
        linear_estimated_total_time_ = linear_distance_ / interpolation_velocity_;

        linear_estimated_time_ = 0.0;

    } else
    {
       // Check for remaining distance in meters
        if(linear_distance_ <= position_tolerance_in_m_)
        {
            //reset scaling
            linear_eta_ = 0;

            //reset elapsed time
            linear_estimated_time_ = 0.0;

            //return
            return linear_target_position_;
        } else
        {

            linear_estimated_time_ += sampling_time_;

            //do the interpolation
            //use time to scale eta from 0 to 1
            linear_eta_ = linear_estimated_time_ / linear_estimated_total_time_;

            //calculate remaining distance to target
            linear_distance_ = (target_position - current_position).norm();

            LOG_DEBUG( "[AbstractKinematics]: interpolate eta:%f, estimateTotalTime:%f", linear_eta_, linear_estimated_total_time_);

            if(linear_eta_ < 0)
            {
                linear_eta_ = 0;
                LOG_WARN_S << "[AbstractKinematics]: interpolate ERROR...eta below zero! Check algorithm.";
            }
            else if(linear_eta_ >1)
            {
                linear_eta_ = 1;
                LOG_WARN_S << "[AbstractKinematics]: interpolate Eta above one. Maybe traverse took longer than expected? elapsedTime: " 
                            << linear_estimated_time_ << " expected time: " << linear_estimated_total_time_;
            }
        }
    }

    //write back the remaining distance
    remaining_distance = linear_distance_;
    
    //retrun the calculation of the new interpolated target position
    return ((linear_start_position_ * ( 1 - linear_eta_)) + (target_position * linear_eta_ ));
}

}
