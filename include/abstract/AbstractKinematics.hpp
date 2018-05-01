#ifndef ABSTRACTKINEMATIC_HPP_
#define ABSTRACTKINEMATIC_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>
#include <base/Eigen.hpp>
#include "KinematicsConfig.hpp"

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
    * @param base_link Target Position is given respect to this link
    * @param target_position Desired position for the target link
    * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
    * @param joint_status Contains current joint angles.
    * @param solution Inverse solution
    * @param solver_status Solution status or error code
    * @return true if a inverse solution was found or else return false
    */
    virtual bool getIK( const std::string &base_link,
                        const base::Vector3d &target_position,
                        const base::Quaterniond &target_orientation,
                        const std::vector<double> &joint_status,
                        std::vector<double> &solution,
                        KinematicsStatus &solver_status) = 0;

    /**
    * @brief Calculate pose of a robot given its joint angles
    * @param base_link FK will be calculated with respect to this link
    * @param target_link FK will be calculated till this target_link
    * @param joint_angles joint angles of the robot
    * @param fk_position fk position
    * @param fk_orientation fk orienation in quaternion
    * @param solver_status Solution status or error code
    * @return true if a forward solution was found or else return false
    */
    virtual bool getFK( const std::string &base_link,
                        const std::string &target_link,
                        const std::vector<double> &joint_angles,
                        base::Vector3d &fk_position,
                        base::Quaterniond &fk_orientation,
                        KinematicsStatus &solver_status) = 0;
    
};

typedef std::shared_ptr<kinematics_library::AbstractKinematics> AbstractKinematicPtr;
//typedef boost::shared_ptr<const AbstractKinematic> AbstractKinematicConstPtr;

};

#endif
