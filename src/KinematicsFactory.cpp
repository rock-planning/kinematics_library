#include "KinematicsFactory.hpp"

namespace kinematics_library
{

KinematicsFactory::KinematicsFactory()
{}

KinematicsFactory::~KinematicsFactory()
{}

RobotKinematicsPtr KinematicsFactory::getKinematicsSolver(KinematicsConfig kinematics_config, KDL::Tree kinematics_KDLTree)
{
	RobotKinematicsPtr kinematic_solver = NULL;

	switch(kinematics_config.robot)
	{	
	    case ARTEMIS:
	    {
		kinematic_solver = std::shared_ptr<artemis::ArtemisKinematics>(new artemis::ArtemisKinematics(kinematics_config, kinematics_KDLTree ));
		break;
	    }
	    default:
	    {
		    std::cout<<"No kinematic solver selected"<<std::endl;
		    return NULL;
	    }
	}
	return kinematic_solver;
}
   

}
