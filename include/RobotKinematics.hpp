#ifndef _ROBOTKINEMATICS_HPP_
#define _ROBOTKINEMATICS_HPP_

#include <iostream>
#include <deque>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
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
            RobotKinematics(KinematicsConfig _kinematicsconfig);
            ~RobotKinematics();;
	    
	    bool initialise(KinematicsStatus &kinematics_status);
            /**
            * Print a welcome to stdout
            * \return nothing
            */
            bool solveIK(const base::samples::RigidBodyState &target_pose,
                         const base::samples::Joints &joint_status,
                         base::commands::Joints &solution,
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
            std::string kin_base_name_, kin_tip_name_;
	    base::samples::RigidBodyState kinematic_pose_;
	    urdf::ModelInterfaceSharedPtr urdf_model_;
	    std::vector< std::pair<double, double> > joints_limits_;
	    void transformFrame( const KDL::Tree &kdl_tree, const std::string &base_link, const std::string &tip_link, KDL::Frame &pose);
	    bool initiailiseURDF(std::string urdf_file);
            

    };
    typedef std::shared_ptr<kinematics_library::RobotKinematics> RobotKinematicsPtr;

} // end namespace trajectory_optimization

#endif // _ROBOTKINEMATICS_HPP_
