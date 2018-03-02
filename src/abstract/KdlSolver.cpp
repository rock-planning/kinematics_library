#include <abstract/KdlSolver.hpp>

namespace kinematics_library
{

    //KdlSolver::KdlSolver(const KDL::Chain _kdl_chain):kdl_chain(_kdl_chain)
    KdlSolver::KdlSolver(const std::size_t _number_of_joints, const std::vector< std::pair<double, double> > _jts_limits, const KDL::Chain _kdl_chain):
    number_of_joints(_number_of_joints), jts_limits(_jts_limits), kdl_chain(_kdl_chain)
    {
        fk_solverPos        = new KDL::ChainFkSolverPos_recursive(kdl_chain);
        ik_solverVelPinv    = new KDL::ChainIkSolverVel_pinv(kdl_chain);

        getJointLimits(min_jtLimits, max_jtLimits);

        maxiter = 150;  //Maximum 100 iterations
        eps=1e-6;       //stop at accuracy 1e-6

        ik_solverPosJL  = new KDL::ChainIkSolverPos_NR_JL(kdl_chain, min_jtLimits, max_jtLimits,
                                                          *fk_solverPos, *ik_solverVelPinv, maxiter, eps);


        kdl_jtArray.data.resize(number_of_joints);
        kdl_ik_jtArray.data.resize(number_of_joints);

    }

    KdlSolver::~KdlSolver()
    {
        if (fk_solverPos)
        {
            fk_solverPos = NULL;
            delete fk_solverPos;
        }

        if (ik_solverPosJL)
        {
            ik_solverPosJL = NULL;
            delete ik_solverPosJL;
        }

        if (ik_solverVelPinv)
        {
            ik_solverVelPinv = NULL;
            delete ik_solverVelPinv;
        }
    }

    bool KdlSolver::getIK(  const std::string &base_link,
                            const base::Vector3d &target_position,
                            const base::Quaterniond &target_orientation,
                            const std::vector<double> &joint_status,
                            std::vector<double> &solution,
                            KinematicsStatus &solver_status)
    {

        convertVectorToKDLArray(joint_status, kdl_jtArray);

        kdl_frame.p.data[0] = target_position(0);
        kdl_frame.p.data[1] = target_position(1);
        kdl_frame.p.data[2] = target_position(2);

        kdl_frame.M = KDL::Rotation::Quaternion(target_orientation.x(), target_orientation.y(),
                                                target_orientation.z(), target_orientation.w() );


        std::cout<<"IK found  "<<ik_solverPosJL->CartToJnt(kdl_jtArray, kdl_frame, kdl_ik_jtArray)<<std::endl;

        int res = ik_solverPosJL->CartToJnt(kdl_jtArray, kdl_frame, kdl_ik_jtArray);
        if( res >= 0)
        {
            convertKDLArrayToVector(kdl_ik_jtArray, solution);
            solver_status.statuscode = KinematicsStatus::IK_FOUND;
            return true;
        }
        else if(res == -3)
        {
            solver_status.statuscode = KinematicsStatus::IK_TIMEOUT;
            return false;
        }
        else
        {
            solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
            return false;
        }



    }

    bool KdlSolver::getFK( const std::string &base_link,
                const std::string &target_link,
                const std::vector<double> &joint_angles,
                base::Vector3d &fk_position,
                base::Quaterniond &fk_orientation,
                KinematicsStatus &solver_status)
    {
        convertVectorToKDLArray(joint_angles, kdl_jtArray);

        if(fk_solverPos->JntToCart(kdl_jtArray, kdl_frame) >= 0)
        {
            fk_position(0) = kdl_frame.p.data[0];
            fk_position(1) = kdl_frame.p.data[1];
            fk_position(2) = kdl_frame.p.data[2];

            kdl_frame.M.GetQuaternion(fk_orientation.x(), fk_orientation.y(), fk_orientation.z(), fk_orientation.w());

            solver_status.statuscode = KinematicsStatus::FK_FOUND;
            return true;
        }
        else
        {
            solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
            return false;
        }

    }

    void KdlSolver::convertVectorToKDLArray(const std::vector<double> &joint_angles, KDL::JntArray &kdl_jt_array)
    {
        for(unsigned int i = 0; i <joint_angles.size(); i++  )
            kdl_jt_array.data(i) = joint_angles.at(i);

    }

    void KdlSolver::convertKDLArrayToVector(const KDL::JntArray &kdl_jt_array, std::vector<double> &joint_angles)
    {
        joint_angles.resize(kdl_jt_array.data.size());

        for(unsigned int i = 0; i <kdl_jt_array.data.size(); i++  )
            joint_angles.at(i) = kdl_jt_array.data(i);

    }

    void KdlSolver::getJointLimits(KDL::JntArray &min_jtLimits, KDL::JntArray &max_jtLimits)
    {
        min_jtLimits.resize(number_of_joints);
        max_jtLimits.resize(number_of_joints);

        for(std::size_t i = 0; i < number_of_joints; i++)
        {
            min_jtLimits(i) = jts_limits.at(i).first;
            max_jtLimits(i) = jts_limits.at(i).second;
        }

    }
}
