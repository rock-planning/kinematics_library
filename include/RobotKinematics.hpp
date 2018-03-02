#ifndef _ROBOTKINEMATICS_HPP_
#define _ROBOTKINEMATICS_HPP_

#include <iostream>
#include <deque>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/JointsTrajectory.hpp>
#include <base-logging/Logging.hpp>

#include "abstract/AbstractKinematics.hpp"
#include "abstract/IkFastSolver.hpp"
#include "abstract/Ikfast.h"
#include "abstract/KdlSolver.hpp"
#include "KinematicsConfig.hpp"



namespace kinematics_library
{
    class RobotKinematics
    {
        public:
            RobotKinematics(KinematicsConfig _kinematicsconfig, KDL::Tree _kinematicsKDLTree);
            ~RobotKinematics();
            /**
            * Print a welcome to stdout
            * \return nothing
            */
            bool solveIK(const base::Vector3d &target_position,
                         const base::Quaterniond &target_orientation,
                         const base::samples::Joints &joint_status,
                         base::commands::Joints &solution,
                         KinematicsStatus &solver_status);

            bool solveIK(const std::vector < base::samples::RigidBodyState> &target_poses,
                         const base::samples::Joints &joint_status,
                         base::JointsTrajectory &solutions,
                         KinematicsStatus &solver_status);

			bool solveIKRelatively (const base::samples::Joints &joint_angles,
                                    const base::samples::RigidBodyState &relative_pose,
			                        base::commands::Joints &solution,
			                        KinematicsStatus &solver_status);

            void solveFK(const base::samples::Joints &joint_angles,
                         base::samples::RigidBodyState &result_pose,
                         KinematicsStatus &solver_status);

        protected:
            boost::shared_ptr<AbstractKinematics> kinematics_solver_;
            KinematicsConfig kinematics_config_;
	    	ikfast::IkFastFunctions<IkReal> ik_;

            /** \brief kdl tree containing joints and links of the robot */
            KDL::Tree kdl_tree_;
            /** \brief kdl chain  containing joints and links of the robot */
            KDL::Chain kdl_chain_;
            /** \brief kdl chain  containing joints and links of the robot */
            KDL::Chain rev_jt_kdlchain_;

        private:
            std::vector<double>current_jt_status_, ik_solution_;
            std::vector<std::string> jt_names_;
            std::string root_name_, base_name_, tip_name_;
			base::Vector3d fk_position_;
	        base::Quaterniond fk_orientation_;
			KDL::Frame transform_pose_;
			void transformFrame( const KDL::Tree &kdl_tree, const std::string &base_link, const std::string &tip_link, KDL::Frame &pose);
            std::vector< std::pair<double, double> > joints_limits_;
		

    };

} // end namespace trajectory_optimization

#endif // _ROBOTKINEMATICS_HPP_
