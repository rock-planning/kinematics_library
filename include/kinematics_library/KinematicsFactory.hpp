#ifndef KINEMATICSFACTORY_HPP_
#define KINEMATICSFACTORY_HPP_

#include <vector>
#include <string>
#include <dlfcn.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/JointsTrajectory.hpp>
#include <base-logging/Logging.hpp>

#include "abstract/AbstractKinematics.hpp"
#include "HandleKinematicConfig.hpp"

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

        AbstractKinematicPtr getKinematicsSolver(const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status);

        bool initialise(const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status);

        KDL::Tree getKDLTree(){return kdl_tree_;}

        KDL::Chain getKDLChain(){return kdl_chain_;}

        KDL::Chain getKDLKinematicsChain(){return kinematics_kdl_chain_;}

        std::vector< std::pair<double, double> > getJointLimits(){return joints_limits_;}

    private:        

        bool initiailiseURDF(std::string urdf_file);

        /** \brief kdl tree containing joints and links of the robot */
        KDL::Tree kdl_tree_;
        /** \brief kdl chain containing joints and links of the robot. This chain might contains fixed joints */
        KDL::Chain kdl_chain_;
        /** \brief kinematics kdl chain containing non-fixed joints and their links of the robot */
        KDL::Chain kinematics_kdl_chain_;

        std::vector<std::string> jt_names_;
        urdf::ModelInterfaceSharedPtr urdf_model_;		
        std::vector< std::pair<double, double> > joints_limits_;
};
};

#endif

