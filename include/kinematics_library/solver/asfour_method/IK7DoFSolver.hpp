/*************************************************************************/
/*  "Analytical Inverse Kinematic Computation for 7-DOF  Manipulator "   */
/*   (Works only for a specific joints combination )                     */
/*   See the paper in the folder for joints configuration                */
/*                                                                       */
/*  This method is based on a paper proposed by                          */
/*  - T. Asfour and r. Dillmann "Human-like Motion of a Humanoid Robot   */
/*    Arm Based on a Closed-Form Solution of the Inverse Kinematics      */
/*    Problem"                                                           */
/*                                                                       */
/*                                                                       */
/*  Alexander Dettmann, Rohit Menon                                      */
/*  DFKI - BREMEN 2019                                                   */
/*************************************************************************/

#pragma once

#include <vector>
#include <boost/function.hpp>
#include <string>

#include <Eigen/Core>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

#include "kinematics_library/abstract/AbstractKinematics.hpp"
#include "kinematics_library/abstract/KinematicsHelper.hpp"

namespace kinematics_library
{
struct SolutionWithScore
{
    base::samples::Joints joints_value;
    double score;
    
    SolutionWithScore(const base::samples::Joints joints_value_, const double score_):joints_value(joints_value_), score(score_){}
};

struct CompareSolutions
{
    bool operator()(SolutionWithScore const &a, SolutionWithScore const &b) {
        return a.score < b.score;
    }
    
};  


enum ZE_MODE
{
    AUTO_ZSTABLE = 0,
    AUTO_ZMAX,
    AUTO_YMAX,
    MANUAL
};


struct IK7DoFConfig
{
    IK7DoFConfig(): ze_mode(AUTO_ZSTABLE), offset_base_shoulder(0.0), offset_shoulder_elbow(0.0), offset_elbow_wrist(0.0), 
                          offset_wrist_tool(0.0){}
    
    ZE_MODE ze_mode;
    double offset_base_shoulder;   // distance between base to shoulder
    double offset_shoulder_elbow;  // distance between shoulder to elbow
    double offset_elbow_wrist;     // distance between elbow to wrist
    double offset_wrist_tool;      // distance between wrist to tool
    std::vector<double> theta_offsets;
    std::vector<double> link_twists;
    std::vector<int> joints_mapping;
    std::vector<std::string> joint_names;
    
};
struct Arm
{
    // Denavit-Hartenberg Parameters
    double dh_d[8];                 // translation along the z-axis in m
    double dh_a[8];                 // translation along the rotated x-axis in m
    double dh_ca[8];                // cosine of the rotation around the rotated x-axis
    double dh_sa[8];                // sine of the rotation around the rotated x-axis
    double dh_do[8];                // offset on top of the rotation angle delta around the z-axis
    


    
    // angle limits
    double j_min[7];       // in rad
    double j_max[7];       // in rad
    
    // parameters to influence the behavior
    ZE_MODE ze_mode;         
    double ze_in;                   // desired z-coordinate of the elbow for manual mode
    double ze_out;                  // chosen z-coordinate of the elbow for manual mode
    double ze_min;                  // calculated minimum z-coordinate for the elbow
    double ze_max;                  // calculated minimum z-coordinate for the elbow
    double ja_last[7];              // the actual joint angles, which are needed to choose a solution which is closest
    
    // inputs, outputs
    double ja_fk_in[7];             // joint angles for the forward kinematics in rad
    double T_Base_2_TCP_fk_out[16]; // resulting transformation matrix from the forward kinematics
    double pos_ik_in[3];            // [X Y Z] in m
    double rot_ik_in[3];            // [RZ, RY, RX] as zyx-Euler angles in rad
    double ja_ik_out[7];   // the resulting joint angles in rad from the inverse kinematics
    double ja_all[8][7];   // all ik solutions
    
    double Rbase2tcp[9];
    
    double T_Base_2_T0[16];
    Eigen::MatrixXd T;
    std::string robot_name;
};
class Ik7DoFSolver: public AbstractKinematics
{
    
public:
    /**
     * @brief  constructor
     */
    Ik7DoFSolver(const KinematicsConfig &kinematics_config,  const std::vector<std::pair<double, double> > &jts_limits, 
                 const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain);
    
    /**
     * @brief  destructor
     */
    ~Ik7DoFSolver();
    
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
    KDL::Frame kdl_frame_;
    KDL::JntArray kdl_jt_array_, kdl_ik_jt_array_;
    std::string base_link_, tip_link_;
    std::string  urdf_file_path_;
    std::vector<std::pair<double, double>> jts_limits_;
    
    Arm* arm_;
    IK7DoFConfig ik7dof_config_;
    
    int initializeArm();
    int fkArm();
    int ikArm();
    
    base::samples::RigidBodyState getRBSPose(double*);
    
    void setDesiredPose(const base::samples::RigidBodyState& rbs_pose, double* pos_vec, double* rot_mat);
    base::samples::Joints listJointsInDesiredOrder( double* joint_values, const std::vector<std::string> &desired_joint_names_order, const std::vector<std::string> &actual_joint_names_order);
    bool validateJointLimits(const base::samples::Joints& joint_values, const std::vector<std::pair<double, double> > &jts_limits);
    std::vector<base::samples::Joints> pickOptimalSolution(const std::vector<base::samples::Joints>& solution_list, const base::samples::Joints& joints_actual);
    
};
}

