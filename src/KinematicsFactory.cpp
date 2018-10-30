#include "KinematicsFactory.hpp"
#include <stdio.h>
#include <stdlib.h>

namespace kinematics_library
{

KinematicsFactory::KinematicsFactory()
{}

KinematicsFactory::~KinematicsFactory()
{}

AbstractKinematicPtr KinematicsFactory::getKinematicsSolver ( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status )
{
    AbstractKinematicPtr kinematic_solver = NULL;

    if ( !initialise ( kinematics_config, kinematics_status ) )     
        return NULL;
    

    switch ( kinematics_config.kinematic_solver )
    {
        case IKFAST:
        {
            LOG_INFO_S<<"[KinematicsFactory]: IKFAST solver is selected";

            if ( !getIKFASTFunctionPtr ( kinematics_config.ikfast_lib, kinematics_status, computeFkFn, computeIkFn ) )             
                return NULL;

            kinematic_solver = std::shared_ptr<IkFastSolver> ( new IkFastSolver ( kinematics_config, joints_limits_, kdl_tree_, rev_jt_kdlchain_, computeFkFn, computeIkFn ) );
            break;
        }
        case KDL:
        {
            LOG_INFO_S<<"[KinematicsFactory]: KDL solver is selected";
            kinematic_solver = std::shared_ptr<KdlSolver> ( new KdlSolver ( kinematics_config, joints_limits_, kdl_tree_, rev_jt_kdlchain_ ) );
            break;
        }
        case TRACIK:
        {
            LOG_INFO_S<<"[KinematicsFactory]: TRACIK solver is selected";
            #if(TRAC_IK_LIB_FOUND)
                kinematic_solver = std::shared_ptr<TracIkSolver> ( new TracIkSolver ( kinematics_config, kdl_tree_, rev_jt_kdlchain_ ) );
            #else
                LOG_FATAL_S << "[KinematicsFactory]: TRACIK is not installed. Please select an another solver !";
                return NULL;
            #endif
            break;
        }
        default:
        {
            kinematics_status.statuscode = KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND;
            LOG_ERROR ( "[KinematicsFactory]: This  kinematicSolver is not available" );
            throw new std::runtime_error ( "This kinematicSolver is not available" );
            return NULL;
        }
    }
    return kinematic_solver;
}

bool KinematicsFactory::initialise ( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status )
{
    LOG_DEBUG ( "[KinematicsFactory]: Initialising kinematic factory d%s", kinematics_config.urdf_file.c_str() );

    if ( !kdl_parser::treeFromFile ( kinematics_config.urdf_file, kdl_tree_ ) )
    {
        kinematics_status.statuscode = KinematicsStatus::KDL_TREE_FAILED;
        LOG_FATAL_S<<"[KinematicsFactory]: Error while initialzing KDL tree";
    }

    if ( !kdl_tree_.getChain ( kinematics_config.base_name, kinematics_config.tip_name, kdl_chain_ ) )
    {
        kinematics_status.statuscode = KinematicsStatus::KDL_CHAIN_FAILED;
        LOG_FATAL ( "[KinematicsFactory]: Could not initiailise KDL chain !" );
        return false;
    }
    else
    {
        LOG_INFO ( "[KinematicsFactory]: KDL chain initialised with size %d",kdl_chain_.segments.size() );
    }

    // clearing the segment of the kdl chain is not clearing no of joints.
    // So create a new chain
    rev_jt_kdlchain_ = KDL::Chain();

    for ( std::size_t i=0; i<kdl_chain_.segments.size(); i++ )
    {
        if ( ! ( kdl_chain_.getSegment ( i ).getJoint().getType() ==KDL::Joint::None ) )        
            rev_jt_kdlchain_.addSegment ( kdl_chain_.getSegment ( i ) );
    }

    if ( !initiailiseURDF ( kinematics_config.urdf_file ) ) 
    {
        kinematics_status.statuscode = KinematicsStatus::URDF_FAILED;
        return true;
    }

    //pack the joint limit as std::pair
    joints_limits_.clear();
    for ( std::size_t jn = 0; jn < rev_jt_kdlchain_.getNrOfJoints(); jn++ )
    {
        joints_limits_.push_back ( std::make_pair ( urdf_model_->getJoint ( rev_jt_kdlchain_.getSegment ( jn ).getJoint().getName() )->limits->lower,
                                   urdf_model_->getJoint ( rev_jt_kdlchain_.getSegment ( jn ).getJoint().getName() )->limits->upper ) );
    }

    LOG_DEBUG ( "[KinematicsFactory]: Initiailising finished" );
    return true;
}

bool KinematicsFactory::initiailiseURDF ( std::string urdf_file )
{
    std::string xml_string;
    std::fstream xml_file ( urdf_file.c_str(), std::fstream::in );

    if ( xml_file.is_open() )
    {
        while ( xml_file.good() )
        {
            std::string line;
            std::getline ( xml_file, line );
            xml_string += ( line + "\n" );
        }
        xml_file.close();
        urdf_model_ = urdf::parseURDF ( xml_string );
        if ( urdf_model_.get() == NULL )
        {
            LOG_ERROR ( "[KinematicsFactory] Error while getting urdf model. urdf_model is empty" );
            return false;
        }
    }
    else
    {
        LOG_ERROR ( "[KinematicsFactory] Cannot open urdf file." );
        return false;
    }
    return true;
}

bool KinematicsFactory::getIKFASTFunctionPtr (  const std::string ikfast_lib, KinematicsStatus &kinematics_status, void ( *ComputeFkFn ) ( const IkReal* j, IkReal* eetrans, IkReal* eerot ),
                                                bool ( *ComputeIkFn ) ( const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions ) )
{
    void *handle;
    char *error;

    handle = dlopen ( ikfast_lib.c_str(), RTLD_LAZY );
    if ( !handle )
    {
        LOG_ERROR ( "[KinematicsFactory]: Cannot open ikfst shared library. Error %s",dlerror() );
        kinematics_status.statuscode = KinematicsStatus::IKFAST_LIB_NOT_AVAILABLE;
        return false;
    }

    // get the forward kinematics fuction pointer
    ComputeFkFn= ( void ( * ) ( const IkReal* , IkReal* , IkReal* ) )  dlsym ( handle, "ComputeFk" );

    if ( ( error = dlerror() ) != NULL )
    {
        LOG_ERROR ( "[KinematicsFactory]: Cannot find ComputeFk function. Error %s",dlerror() );
        dlclose ( handle );
        kinematics_status.statuscode = KinematicsStatus::IKFAST_FUNCTION_NOT_FOUND;
        return false;
    }

    // get the inverse kinematics fuction pointer
    ComputeIkFn = ( bool ( * ) ( const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& ) )  dlsym ( handle, "ComputeIk" );
    if ( ( error = dlerror() ) != NULL )
    {
        LOG_ERROR ( "[KinematicsFactory]: Cannot find ComputeIk function. Error %s",dlerror() );
        dlclose ( handle );
        kinematics_status.statuscode = KinematicsStatus::IKFAST_FUNCTION_NOT_FOUND;
        return false;
    }
    return true;
}
}