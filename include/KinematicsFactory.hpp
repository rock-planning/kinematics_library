#ifndef KINEMATICSFACTORY_HPP_
#define KINEMATICSFACTORY_HPP_

#include <vector>
#include <string>
#include "abstract/AbstractKinematics.hpp"
#include "RobotKinematics.hpp"
#include "robots/artemis/ArtemisKinematics.hpp"

namespace kinematics_library
{

/**
 * @class KinematicsFactory
 * @brief Provides a factory class for the AbstractKinematics class.
 */
class KinematicsFactory
{

public:
    /**
    * @brief  constructor
    */
    KinematicsFactory();
    /**
    * @brief  destructor
    */
    ~KinematicsFactory();

     RobotKinematicsPtr getKinematicsSolver(KinematicsConfig kinematics_config);

};

};

#endif

