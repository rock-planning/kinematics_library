/***************************************************************************/
/*  An analytical kinematic solver for a 7-DOF Anthropomorphic Manipulator */
/*  (SRS type - Spherical - Revolute - Spherical Joints)                   */
/*                                                                         */
/*  This solver is based on the paper:                                     */
/*  "Analytical Inverse Kinematic Computation for 7-DOF Redundant          */
/*   Manipulators With Joint Limits and Its Application to Redundancy      */
/*   Resolution" - Masayuki Shimizu, Hiromu kakuya, Woo-Keun Yoon,         */
/*   Kosei Kitagaki, Kazuhiro Kosuge.                                      */
/*                                                                         */
/*                                                                         */
/*  Author: Sankaranarayanan Natarajan                                     */
/*  DFKI - BREMEN 2019                                                     */
/***************************************************************************/

#ifndef _SRSKINEMATICSOLVER_HPP_
#define _SRSKINEMATICSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include<iostream>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <abstract/AbstractKinematics.hpp>
#include <abstract/KinematicsHelper.hpp>
#include <solver/SRSKinematicHelper.hpp>


#define DEBUG 1                 // simply debug stuff. need to be removed soon
#define ZERO std::numeric_limits<double>::epsilon()

namespace kinematics_library
{

/**
 * @class SRSKinematicSolver
 * @brief An analytical kinematics solvers for an anthropomorphic manipulator.
 */
class SRSKinematicSolver : public AbstractKinematics
{

    public:
        /**
        * @brief  constructor
        */
        SRSKinematicSolver(const KinematicsConfig &kinematics_config,  const std::vector<std::pair<double, double> > &jts_limits, 
                           const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain);

        /**
        * @brief  destructor
        */
        ~SRSKinematicSolver();

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

        /*! Calculates the inverse kinematics for a 7-DOF Anthropomorphic Manipulator with Joint Limits
        *   Input  - pos and rot contains information about the desired position and rotation
        *   Output - The joint angles for the pos and rot will be stored in an two dimensional array "jointangles"
        *            jointangles[0][7] - This solution is from one extreme of an arm angle range
        *            jointangles[1][7] - This solution is from other extreme of an arm angle range
        *   Return - 0 if successful, -99 if out of reach or a number between -1 and -6 specifying the single joint which ran into limits
        *            or -66 no armangle feasible due to joint limits
        */
        int invkin(const double pos[3], const double rot[3], double jointangles[7]);

        /*! Calculates the Arm Angle range
        *   Input  - As,Bs,Cs-represent the shoulder and Aw,Bw,Cw represent the wrist and array debg gives which joint had singularity and
        *   Output - A two dimensional array "AA" has the arm angle range
        *            ct_aa is a counter telling the number of arm angle range
        *   Return - 0 if successful or return -1 or -2 or -3 or -5 or -6 or -7 depending on which joint has exceed joint limit
        */
        int cal_armangle(const std::vector<double> &As, const std::vector<double> &Bs, const std::vector<double> &Cs,
                            const std::vector<double> &Aw, const std::vector<double> &Bw, const std::vector<double> &Cw,
                            std::vector< std::pair<double,double>  > &final_feasible_armangle);

        int tangenttype_armangle(const double &an, const double bn, const double cn,
                                    const double &ad, const double &bd, const double &cd,
                                    const double &at, const double &bt, const double &ct,
                                    const double &condition, const int &jointnumber,
                                    const double &min_jointlimit, const double &max_jointlimit,
                                    const std::string& jointname);

        int feasible_armangle_monotonicfunction(const double &an, const double &bn, const double &cn,
                                                    const double &ad, const double &bd, const double &cd,
                                                    const double &min_jtag, const double &max_jtag,
                                                    const int &jointnumber, const std::string& jointname);

        int feasible_armangle_tangenttype_cyclicfunction(const double &an, const double &bn, const double &cn,
                                                                            const double &ad, const double &bd, const double &cd,
                                                                            const double &at, const double &bt, const double &ct,
                                                                            const double &min_jtag, const double &max_jtag,
                                                                            const int &jointnumber, const std::string &jointname);


        int feasible_armangle_tangenttype_stationarycase(const double &an, const double &bn, const double &cn,
                                                                            const double &ad, const double &bd, const double &cd,
                                                                            const double &at, const double &bt, const double &ct,
                                                                            const int &jointnumber,const std::string& jointname);

        int feasible_armangle_cosinetype_cyclicfunction(const double &at, const double &bt, const double &ct,
                                                        const double &min_jtag, const double &max_jtag,
                                                        const int &jointnumber, const std::string& jointname);

        int calculate_region_armAngle_tangentcylic(const double &an, const double &bn, const double &cn,
                                                    const double &ad, const double &bd, const double &cd,
                                                    const double &joint_limit, std::pair< double,double > &psi_pair );


        int solve_transcendental_equation(const double a, const double b, const double c, std::pair<double, double>& transcendental_solutions);
        int solve_transcendental_equation_newmethod(const double a, const double b, const double c, std::pair<double, double>& transcendental_solutions);

        int psi_picker(std::pair<double, double> act_psi, const std::string& limit, double &result);
        int check_psi_range_bw_negPI_posPI(std::pair<double, double> &act_psi);


    private:
        KDL::ChainFkSolverPos_recursive *fk_kdlsolver_pos_;

        KDL::Frame kdl_frame_;
        KDL::JntArray kdl_jt_array_, kdl_ik_jt_array_;
        std::string base_link_, tip_link_;
        std::string  urdf_file_path_;
        
        double DBS;                                     // distance between base to shoulder
        double DSE;                                     // distance between shoulder to elbow
        double DEW;                                     // distance between elbow to wrist
        double DWT;                                     // distance between wrist to tool
        std::vector<double> l_bs, l_se, l_ew, l_wt;     // vector holding the link length
        
        
        
        //helper function
        void save_tangent_joint_function(const double &an, const double &bn, const double &cn,
                                            const double &ad, const double &bd, const double &cd,
                                            const double &min_jtag, const double &max_jtag, const char *outputdata_file);

        void save_cosine_joint_function(const double &a, const double &b, const double &c,
                                        const double &min_jtag, const double &max_jtag, const char *outputdata_file);

        void save_psi_file();
        void save_psi_file_helper(const char* outputdata_file, double min, double max, double min_jt, double max_jt);
        
        std::string printError(int err);
        
//         double dist_base_shoulder;              // DBS - distance between base to shoulder
//         double dist_shoulder_elbow;             // DSE - distance between shoulder to elbow
//         double dist_elbow_wrist;                // DEW - distance between elbow to wrist
//         double dist_wrist_tool;                 // DWT - distance between wrist to tool
// //         std::vector<double> l_bs, l_se, l_ew, l_wt;     // vector holding the link length
//         std::vector<double> link_base_shoulder, link_shoulder_elbow, link_elbow_wrist, link_wrist_tool;     // vector holding the link length
//         double link_offset[7];                              // link offsets

        double min_j1, max_j1;                          // minimum and maximum values for joint 1
        double min_j2, max_j2;                          // minimum and maximum values for joint 2
        double min_j3, max_j3;                          // minimum and maximum values for joint 3
        double min_j5, max_j5;                          // minimum and maximum values for joint 5
        double max_j6, min_j6;                          // minimum and maximum values for joint 6
        double max_j7, min_j7;                          // minimum and maximum values for joint 7

        std::vector< ArmAngle > feasible_psi;
        std::vector< ArmAngle > infeasible_psi;

protected:
    int ERR_REACH;
};
}
#endif





