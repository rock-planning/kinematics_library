/************************************************************************************************************/
/* HybridIkSolver is an inverse kinematic solver for a highly redundant robot. The solver combines the      */
/* analytical/numerical solver with an optimization-based solver to solve the kinematic problem. The robot  */
/* is represented as passive and active chains. The passive chain is a kinematic chain for which the        */
/* analytical/numerical solver is available and the active chain is optimized to reach the goal. For the    */
/* active chain, the inverse kinematic problem is formualted as a weighted sum optimization problem.        */  
/*                                                                                                          */
/*  Author: Sankaranarayanan Natarajan                                                                      */
/*  DFKI - BREMEN 2021                                                                                      */
/************************************************************************************************************/

#ifndef _HYBRID_IK_HPP_
#define _HYBRID_IK_HPP_

#include <cmath>
#include <iostream>
#include <chrono>
#include <nlopt.hpp>


#include <kinematics_library/KinematicsFactory.hpp>
#include <kinematics_library/abstract/AbstractKinematics.hpp>
#include <kinematics_library/solver/optimization_method/OptConfig.hpp>

namespace kinematics_library
{
/**
 * @class HybridIkSolver
 * @brief Kinematics solvers for solving different task.
 */

class HybridIkSolver : public kinematics_library::AbstractKinematics
{
    typedef std::vector< base::commands::Joints > IkSolutions;
    typedef std::vector< base::commands::Joints > IkSolutionsVec;
    typedef std::vector< KDL::Frame > KDLFrameVec;

    public:
        /**
        * @brief  constructor
        */
        HybridIkSolver( const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree, 
                        const KDL::Chain &kdl_kinematic_chain, const KDL::Chain &kdl_chain);
        /**
        * @brief  destructor
        */
        ~HybridIkSolver();

        bool loadKinematicConfig( const kinematics_library::KinematicsConfig &kinematics_config, kinematics_library::KinematicsStatus &kinematics_status);

        bool getOptParamConfig(const YAML::Node &yaml_data, OptParamConfig &config);

        bool initialiseSolver ( const KinematicsConfig &kinematics_config, 
                                kinematics_library::KinematicsStatus &kinematics_status );

        /**
        * @brief Calculate the joint angles for a manipulator to reach a desired Pose.
        * @param target_position Desired position for the target link
        * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
        * @param joint_status Contains current joint angles.
        * @param solution Inverse solution
        * @param solver_status Solution status or error code
        * @return true if a inverse solution was found or else return false
        */
        bool solveIK(const base::samples::RigidBodyState &target_pose, const base::samples::Joints &joint_status,
                     std::vector<base::commands::Joints> &solution, kinematics_library::KinematicsStatus &solver_status);

        /**
        * @brief Calculate pose of a manipulator given its joint angles        
        * @param joint_angles joint angles of the manipulator
        * @param fk_position fk position
        * @param fk_orientation fk orienation in quaternion
        * @param solver_status Solution status or error code
        * @return true if a forward solution was found or else return false
        */
        bool solveFK(const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, kinematics_library::KinematicsStatus &solver_status);
        
        void getChainSegementPose(const base::samples::Joints &joint_angles, KDLFrameVec &segement_pose){}

        void calculateNodeChainFK(const std::vector<double> &opt_jt_ang, KDL::Frame &node_pose, KDL::Frame &passive_pose);
        void calculateNodeChainFKGrad(const std::vector<double> &opt_jt_ang, KDLFrameVec &node_pose_grad, KDLFrameVec &passive_pose_grad);
        void calculateFK(const std::vector<double> &opt_jt_ang);

        double calculateIKCost(const KDL::Frame &chain_pose, const KDL::Frame &node_pose, IkSolutions &ik_solution);
        double calculateOverallIKCost(const std::vector<double>& opt_jt_ang, std::vector<double>& grad);
        void getJointsLimitsConstraintCost( const unsigned &constraints_size, const unsigned &x_size,
                                            const double* x, double *result, double* grad);
        double jointMovementCost(const std::vector<double> &opt_jt_ang,  std::vector<double>& grad);
        void calculateJointGradient(const std::vector<double> &opt_jt_ang);
        double ik_cost_, joints_mov_cost_, overall_costs_;

        inline static double fRand(double min, double max)
    {
      double f = (double)rand() / RAND_MAX;
      return min + f * (max - min);
    }

    private:

        KDL::ChainFkSolverPos_recursive *fk_kdlsolver_pos_;
        KDL::Chain kdl_kinematic_chain_;
        KDL::JntArray kdl_jt_array_;
        KDL::Frame kdl_frame_;

        nlopt::opt nlopt_solver_;
        std::vector<double> opt_var_;
        std::vector<std::pair<double,double> > active_chain_jts_limits_;
        std::vector< double > jts_lower_limit_, jts_upper_limit_;
        OptParamConfig opt_param_;
        CostsWeight chain_costs_weight_;
        kinematics_library::AbstractKinematicPtr passive_chain_kin_solver_;
        KDL::ChainFkSolverPos_recursive* passive_chain_fk_solver_;
        KDL::Frame passive_chain_target_pose_;
        KDL::Chain active_chain_, passive_chain_;
        KDL::Frame passive_chain_pose_, passive_full_chain_pose_;
        KDLFrameVec passive_chain_pose_grad_;

        size_t num_active_joints_;

        // This are fk solvers for passive chain
        float jump_;
        std::vector<std::vector<double>> jt_ang_grad_;
        base::samples::Joints  passive_chain_joint_status_;
        IkSolutionsVec passive_chain_ik_solutions_;
        std::vector<IkSolutionsVec> passive_chain_ik_solutions_grad_;
        // These are fk solvers from active chain base link to passive chain base links
        // Reason for having this full chain is: There could be fixed joints between the active and passive joints
        KDL::ChainFkSolverPos_recursive* node_chain_fk_solver_;
        KDL::Chain node_chain_;
        KDL::Frame node_chain_fk_pose_;
        KDLFrameVec node_chain_fk_pose_grad_;
        std::vector<std::uniform_real_distribution<double>> random_dist_;
        std::mt19937 random_gen_; //Standard mersenne_twister_engine seeded with rd()

        void initialiseProblem(const OptParamConfig &problem_param);        

        bool getCostsWeightConfig(const YAML::Node &yaml_data, const std::string &chain, CostsWeight &weight);        

        void getJointLimits(const std::vector<std::pair<double, double> > jts_limits, 
                            std::vector< double > &lower_limits, std::vector< double > &upper_limits);        

        bool getKinematicChain( const KDL::Tree &kdl_tree, const std::string &base_name, const std::string &tip_name, 
                                kinematics_library::KinematicsStatus &kinematics_status, KDL::Chain &kdl_chain);

        void convertPoseBetweenDifferentFramesFK( const KDL::Tree &kdl_tree, const base::samples::Joints &joint_status, 
                                                  const base::samples::RigidBodyState &source_pose, base::samples::RigidBodyState &target_pose);

        double positionCost( const KDL::Frame& target_pose,  const KDL::Frame& node_pose, const kinematics_library::AbstractKinematicPtr &kinematic_solver, 
                                        const base::samples::Joints &joint_status, kinematics_library::KinematicsStatus &solver_status);

        double positionCost( const KDL::Frame& target_pose, const KDL::Frame& node_pose);
};
}
#endif