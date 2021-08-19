#ifndef _PROBLEM_FORMULATION_HPP_
#define _PROBLEM_FORMULATION_HPP_

#include <cmath>
#include <iostream>
#include <math.h>
#include <string>
#include <memory>

// #include <robot_model/RobotModel.hpp>
#include <kinematics_library/KinematicsConfig.hpp>
#include "kinematics_library/abstract/AbstractKinematics.hpp"
// #include <collision_detection/CollisionFactory.hpp>

//#include <base/samples/RigidBodyState.hpp>
#include <base/JointLimits.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace kinematics_library
{

// the differentiation rules (backward)
static const double DIFF_RULES[3][5] = 
{   
    {11.0/6.0, -3.0,    3.0/2.0,   -1.0/3.0,    0},         // velocity
    {2.0,      -5.0,    4.0,        -1.0,       0},         // acceleration
    {5.0/2.0,  -9.0,    12.0,       -7.0,       3.0/2.0}   // jerk
};

enum DerivType
{
    VELOCITY, ACCELERATION, JERK
};

using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

struct GrooveVariable
{
    double s;
    double c;
    double r;
};

enum CostType
{
    POSITION_COST, ORIENTATION_COST, VELOCITY_COST, ACCELERATION_COST, JERK_COST, MIN_VEL_EE_COST
};

struct ProblemParameters
{
    std::map<CostType, GrooveVariable> groove_param;
    std::map<CostType, double> costs_weight;
    double max_time;
    double max_iter;
    double abs_tol; // absolute tolerance on optimization parameter 
    double rel_tol; // absolute tolerance on optimization parameter 
};

    
class ProblemFormulation
{
public:

    ProblemFormulation ();
    ~ProblemFormulation ();

    bool initialise(const ProblemParameters &problem_param, const KDL::Chain &kdl_kinematic_chain, const std::vector<std::pair<double, double> > &jts_limits);    

    void assignTarget(const base::samples::RigidBodyState &target_pose, const std::vector<double> &cur_jts);

    double getPosistionCost(const KDL::Frame& fk_pose, std::vector<double>& grad);
    double getOrientationCost(const KDL::Frame& fk_pose, std::vector<double>& grad);
    double getVelocityCost(const std::vector<double>& x, std::vector<double>& grad);
    double getAccelerationCost(const std::vector<double>& x, std::vector<double>& grad);
    double getJerkCost(const std::vector<double>& x, std::vector<double>& grad);
    double getMinEEVelCost(const std::vector<double>& x, std::vector<double>& grad);

    double grooveFunction(const GrooveVariable &groove_var, const double &cost);
    double grooveDerivativeFunction(const GrooveVariable &groove_var, const double &cost);

    void calculateDerivatives(const std::vector<double>& x);

    double getOverallCost(const std::vector<double>& x, std::vector<double>& grad);

    void getJointsLimitsConstraintCost( const unsigned &constraints_size, const unsigned &x_size,
                                        const double* x, double *result, double* grad);

    KDL::Frame target_pose_;
    void assingGoal(const KDL::Frame &frame);

private:

    KDL::Frame target_frame_, kdl_frame_;
    KDL::ChainFkSolverPos_recursive *fk_kdlsolver_pos_;
    KDL::JntArray kdl_jt_array_;
    KDL::Chain kdl_kinematic_chain_;

    ProblemParameters problem_param_;
    GrooveVariable pos_groove_var_, ort_groove_var_, vel_groove_var_, acc_groove_var_, jerk_groove_var_;
    GrooveVariable min_vel_groove_var_;
    double pos_cost_weight_, ort_cost_weight_, vel_cost_weight_, acc_cost_weight_, jerk_cost_weight_;
    base::samples::RigidBodyState opt_fk_pose_;
    kinematics_library::KinematicsStatus kinematic_status_;
    std::vector<std::string> joints_name_;
    // variable to store the delta jump in joint space and cartesian space
    std::vector<KDL::Frame> fk_jac_; 
    std::vector<std::vector<double>> jtang_jac_;
    Eigen::MatrixXd jt_vel_;
    Eigen::MatrixXd jt_acc_;
    std::vector<double> jts_lower_limit_, jts_upper_limit_;
    float jump_;
    size_t opt_var_size_;
    std::vector<std::vector<double>> prev_jtang_;
    std::vector<std::pair<double, double> > jts_limits_;
    kinematics_library::AbstractKinematicPtr kin_solver_;

    void calculateFK(const std::vector<double> &opt_jt_ang, KDL::JntArray &kdl_jt_array, KDL::Frame &kdl_frame);
    void calculateJacobian(const std::vector<double>& x);
    double getQuaternionDiff(const KDL::Rotation& rot_1, const KDL::Rotation& rot_2);    
    Eigen::MatrixXd getDerivative(const DerivType &deriv_type, const std::vector<double>& x);
    void storePreviousRobotState(const std::vector<double>& x);
};
}

#endif
