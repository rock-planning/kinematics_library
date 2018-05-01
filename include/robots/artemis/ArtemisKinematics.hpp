#ifndef _ARTEMIS_KINEMATICS_HPP_
#define _ARTEMIS_KINEMATICS_HPP_

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

#include "KinematicsConfig.hpp"
#include "RobotKinematics.hpp"
#include "robots/artemis/artemis_arm_ikfast_solver.h"

namespace artemis
{
    class ArtemisKinematics: public kinematics_library::RobotKinematics
    {
        public:
            ArtemisKinematics(kinematics_library::KinematicsConfig artemis_config,  KDL::Tree kinematicsKDLTree);
            ~ArtemisKinematics(){};
        
        protected:
	    kinematics_library::KinematicsConfig artemis_config_;         

    };

} // end namespace spacebot_kinematics

#endif // _ARTEMIS_KINEMATICS_HPP_
