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
#include "abstract/IkFastSolver.hpp"
#include "abstract/KdlSolver.hpp"
#if TRACK_IK_LIB_FOUND    
    #include "abstract/TrackIkSolver.hpp"
#endif


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
		
	private:
	
		bool initialise(const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status);
		
		bool initiailiseURDF(std::string urdf_file);
		
		bool getIKFASTFunctionPtr(const std::string ikfast_lib, KinematicsStatus &kinematics_status, void (*ComputeFkFn)(const IkReal* j, IkReal* eetrans, IkReal* eerot),
								  bool (*ComputeIkFn)(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions));
		
		/** \brief kdl tree containing joints and links of the robot */
		KDL::Tree kdl_tree_;
		/** \brief kdl chain  containing joints and links of the robot */
		KDL::Chain kdl_chain_;
		/** \brief kdl chain  containing joints and links of the robot */
		KDL::Chain rev_jt_kdlchain_;
		
		std::vector<std::string> jt_names_;
		urdf::ModelInterfaceSharedPtr urdf_model_;		
		std::vector< std::pair<double, double> > joints_limits_;
		
		bool (*computeIkFn)(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions);
		void (*computeFkFn)(const IkReal* j, IkReal* eetrans, IkReal* eerot);

};

};

#endif

