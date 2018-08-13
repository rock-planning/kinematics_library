#include "KinematicsFactory.hpp"

namespace kinematics_library
{

KinematicsFactory::KinematicsFactory()
{}

KinematicsFactory::~KinematicsFactory()
{}

RobotKinematicsPtr KinematicsFactory::getKinematicsSolver(KinematicsConfig kinematics_config)
{
	RobotKinematicsPtr kinematic_solver = NULL;

	switch(kinematics_config.robot)
	{	
	    case ARTEMIS:
	    {
		kinematic_solver = std::shared_ptr<artemis::ArtemisKinematics>(new artemis::ArtemisKinematics(kinematics_config));
		break;
	    }
	    default:
	    {
		LOG_INFO_S<<"[KinematicsFactory]: Unknown robot "<<kinematics_config.robot<<" . So KDL solver will be used";
		kinematics_config.kinematic_solver = kinematics_library::KDL;
		kinematic_solver = std::shared_ptr<RobotKinematics>(new RobotKinematics(kinematics_config));
		break;
	    }
	}
	return kinematic_solver;
}
   

}
