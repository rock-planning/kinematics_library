#ifndef ABSTRACTKINEMATIC_HPP_
#define ABSTRACTKINEMATIC_HPP_

#include <vector>
#include <boost/function.hpp>
#include <iostream>
#include <string>
#include <base/Eigen.hpp>
#include "kinematics_library/KinematicsConfig.hpp"
#include "kinematics_library/abstract/KinematicsHelper.hpp"

/** \file AbstractKinematics.hpp
*    \brief Abstract kinematics header.
*/

namespace kinematics_library
{

/**
 * @class AbstractKinematic
 * @brief Provides an abstract interface for kinematics solvers.
 */
class AbstractKinematics
{

    public:
        /**
        * @brief  constructor
        */
        AbstractKinematics();
        /**
        * @brief  destructor
        */
        virtual ~AbstractKinematics();	

        /**
        * @brief Calculate the joint angles for a robot to reach a desired Pose.    
        * @param target_position Desired position for the target link
        * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
        * @param joint_status Contains current joint angles.
        * @param solution Inverse solution
        * @param solver_status Solution status or error code
        * @return true if a inverse solution was found or else return false
        */
        virtual bool solveIK( const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                              std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status) = 0;

        /**
        * @brief Calculate pose of a robot given its joint angles
        * @param joint_angles joint angles of the robot
        * @param fk_position fk position
        * @param fk_orientation fk orienation in quaternion
        * @param solver_status Solution status or error code
        * @return true if a forward solution was found or else return false
        */
        virtual bool solveFK( const base::samples::Joints &joint_status, base::samples::RigidBodyState &fk_pose,                        
                              KinematicsStatus &solver_status) = 0;

        bool solveIKRelatively( const base::samples::RigidBodyState &current_pose, const base::samples::Joints &joint_angles, 
                                const base::samples::RigidBodyState &relative_pose,
                                std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status);

        bool solveIKLinearly( const base::samples::RigidBodyState &current_pose, const base::samples::RigidBodyState &relative_pose, 
                              const base::samples::Joints &joint_angles, std::vector<base::commands::Joints> &solution, double &remaining_distance,
                              KinematicsStatus &solver_status);

        base::samples::RigidBodyState getRelativePose( const base::samples::RigidBodyState &current_pose, 
                                                       const base::samples::RigidBodyState &relative_pose);

        base::samples::RigidBodyState transformPose(const std::string &frame_name, const base::samples::RigidBodyState &current_pose);

        std::string getWorldRootName(){return kdl_tree_.getRootSegment()->first;}

    protected:
        base::samples::RigidBodyState kinematic_pose_;
        std::vector<double>current_jt_status_, ik_solution_;
        std::vector<std::string> jt_names_;
        std::size_t number_of_joints_;
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;

        void assignVariables(const KinematicsConfig &kinematics_config, const KDL::Chain &kdl_chain);

    private:
        double position_tolerance_in_m_, interpolation_velocity_, sampling_time_;    
        Eigen::Vector3d linear_target_position_, linear_start_position_;
        bool linear_movement_;
        double linear_eta_, linear_distance_, linear_estimated_total_time_, linear_estimated_time_; 

        const Eigen::Vector3d interpolate( const Eigen::Vector3d& current_position, const Eigen::Vector3d& target_position, 
                                        double& remaining_distance );
};

typedef std::shared_ptr<kinematics_library::AbstractKinematics> AbstractKinematicPtr;
};

#endif
