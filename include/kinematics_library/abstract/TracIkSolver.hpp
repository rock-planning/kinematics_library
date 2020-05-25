#ifndef _TRACIKSOLVER_HPP_
#define _TRACIKSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include<iostream>

#include <trac_ik/trac_ik.hpp>

#include "kinematics_library/abstract/AbstractKinematics.hpp"

namespace kinematics_library
{

enum TracIKSolverType
{
    SPEED = 0,
    DISTANCE,
    MANIP1,
    MANIP2
};

struct TracIkConfig
{
    TracIkConfig() : solver_type(SPEED), timeout_sec(0.1), eps(0.001), tolerances(6, 0.0) {}
    
    // Type of solver for trac_ik- selecting speed gives the first ik soln selecting distance tries 
    // to minimize distance between current and solution in given time
    TracIKSolverType solver_type;
    // time out - used if trac ik is available
    double timeout_sec;
    // stopping criteria for the numerical solver. Stop the solver, if the error is below the eps.              
    double eps;
    //End Effector Pose Tolerance - used for setting the tolerances in trac_ik
    std::vector<double> tolerances;
    std::vector <double> joints_err_weight;    

};

/**
 * @class TracIkSolver
 * @brief Kinematics solvers using TracIk.
 */
class TracIkSolver : public AbstractKinematics
{

    public:
        /**
        * @brief  constructor
        */
        TracIkSolver(const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain);

        /**
        * @brief  destructor
        */
        ~TracIkSolver();

        bool loadKinematicConfig( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status);

        /**
        * @brief Calculate the joint angles for a manipulator to reach a desired Pose.
        * @param target_position Desired position for the target link
        * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
        * @param joint_status Contains current joint angles.
        * @param solution Inverse solution
        * @param solver_status Solution status or error code
        * @return true if a inverse solution was found or else return false
        */
        bool solveIK(const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                     std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status);

        /**
        * @brief Calculate pose of a manipulator given its joint angles        
        * @param joint_angles joint angles of the manipulator
        * @param fk_position fk position
        * @param fk_orientation fk orienation in quaternion
        * @param solver_status Solution status or error code
        * @return true if a forward solution was found or else return false
        */
        bool solveFK(const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status);
        
        void getChainSegementPose(const base::samples::Joints &joint_angles,  std::vector<KDL::Frame> &segement_pose){}

    private:
        KDL::ChainFkSolverPos_recursive *fk_kdlsolver_pos_;
        KDL::Chain kdl_kinematic_chain_;
        KDL::Frame kdl_frame_;
        KDL::JntArray kdl_jt_array_, kdl_ik_jt_array_;
        std::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_;
        std::string base_link_, tip_link_;
        std::string  urdf_file_path_;
        unsigned int max_iter_;
        double eps_;
        KDL::Twist bounds;
        
        TracIkConfig trac_ik_config_;
};
}
#endif





