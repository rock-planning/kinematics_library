#ifndef _KDLSOLVER_HPP_
#define _KDLSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include<iostream>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include "AbstractKinematics.hpp"

namespace kinematics_library
{

/**
 * @class AbstractKdlSolver
 * @brief Kinematics solvers using KDL.
 */
class KdlSolver : public AbstractKinematics
{

    public:
        /**
        * @brief  constructor
        */
        KdlSolver(const std::size_t _number_of_joints, const std::vector<std::pair<double, double> > _jts_limits, const KDL::Chain _kdl_chain);

        /**
        * @brief  destructor
        */
        virtual ~KdlSolver();

        /**
        * @brief Calculate the joint angles for a manipulator to reach a desired Pose.
        * @param base_link Target Position is given respect to this link
        * @param target_position Desired position for the target link
        * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
        * @param joint_status Contains current joint angles.
        * @param solution Inverse solution
        * @param solver_status Solution status or error code
        * @return true if a inverse solution was found or else return false
        */
        bool getIK( const std::string &base_link,
                    const base::Vector3d &target_position,
                    const base::Quaterniond &target_orientation,
                    const std::vector<double> &joint_status,
                    std::vector<double> &solution,
                    KinematicsStatus &solver_status);

        /**
        * @brief Calculate pose of a manipulator given its joint angles
        * @param base_link FK will be calculated with respect to this link
        * @param target_link FK will be calculated till this target_link
        * @param joint_angles joint angles of the manipulator
        * @param fk_position fk position
        * @param fk_orientation fk orienation in quaternion
        * @param solver_status Solution status or error code
        * @return true if a forward solution was found or else return false
        */
        bool getFK( const std::string &base_link,
                    const std::string &target_link,
                    const std::vector<double> &joint_angles,
                    base::Vector3d &fk_position,
                    base::Quaterniond &fk_orientation,
                    KinematicsStatus &solver_status);

    private:
        void convertVectorToKDLArray(const std::vector<double> &joint_angles, KDL::JntArray &kdl_jt_array);
        void convertKDLArrayToVector(const KDL::JntArray &kdl_jt_array, std::vector<double> &joint_angles);
        void getJointLimits(KDL::JntArray &min_jtLimits, KDL::JntArray &max_jtLimits);

        std::size_t number_of_joints;
        std::vector< std::pair<double, double> > jts_limits;

        KDL::ChainFkSolverPos_recursive *fk_solverPos;
        KDL::ChainIkSolverPos_NR_JL *ik_solverPosJL;
        KDL::ChainIkSolverVel_pinv *ik_solverVelPinv;
        KDL::Chain kdl_chain;
        KDL::JntArray kdl_jtArray, kdl_ik_jtArray;
        KDL::JntArray min_jtLimits, max_jtLimits;
        KDL::Frame kdl_frame;        
        unsigned int maxiter;
        double eps;
};


}
#endif





