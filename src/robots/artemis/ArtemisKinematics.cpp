#include "robots/artemis/ArtemisKinematics.hpp"
#include <stdio.h>
#include <stdlib.h>

namespace artemis
{

    ArtemisKinematics::ArtemisKinematics(kinematics_library::KinematicsConfig artemis_config,  KDL::Tree kinematicsKDLTree) : 
					artemis_config_(artemis_config), RobotKinematics(artemis_config, kinematicsKDLTree)
    {}
    
}
