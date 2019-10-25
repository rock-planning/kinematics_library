#pragma once

#ifndef SRSKINEMATICS_HELPER_H_
#define SRSKINEMATICS_HELPER_H_

#include <iostream>
#include <vector>
#include <limits>
#include "math.h"
#include <utility>
#include <algorithm>
#include <Eigen/Dense>

// #define PI 3.1415926535897932384626433832795
// #define DTR 0.0174532925    //Degree to Radians
// #define RTD 57.29577951     //Radians to Degree
// #define NAR -0.00000001
// #define PAR  0.00000001
// #define NAC -0.000005
// #define PAC 0.000005
// #define HALFDEGREE 0.008726646


struct SRSKinematic
{
    static constexpr double PI          = 3.1415926535897932384626433832795;
    static constexpr double DTR         = 0.0174532925;    //Degree to Radians
    static constexpr double RTD         = 57.29577951;     //Radians to Degree
    static constexpr double NAR         = -0.00000001;
    static constexpr double PAR         =  0.00000001;
    static constexpr double NAC         = -0.000005;
    static constexpr double PAC         = 0.000005;
    static constexpr double HALFDEGREE  = 0.008726646;  
    static constexpr double ZERO        = std::numeric_limits<double>::epsilon();
    
    enum Errorcode
    {
        
        SUCCESS                 = 0 ,      /* Kinematic success */
        ERR_BOUND_J1            = -1,      /* Boundary condition failed in Joint 1 */
        ERR_BOUND_J2            = -2,      /* Boundary condition failed in Joint 2 */
        ERR_BOUND_J3            = -3,      /* Boundary condition failed in Joint 3 */
        ERR_BOUND_J5            = -5,      /* Boundary condition failed in Joint 5 */
        ERR_BOUND_J6            = -6,      /* Boundary condition failed in Joint 6 */
        ERR_BOUND_J7            = -7,      /* Boundary condition failed in Joint 7 */
        ERR_REACH               = -11,     /* Pose not reachable */
        ERR_TRANS_EQU           = -21,     /* Error in solving the transcendental equation - the sqaure root is complex */
        ERR_TRANS_EQU_COND      = -22,     /* A condition arised while solving the transcendental equation - It need to be studied */
        ERR_TRANS_EQU_PICK_1    = -23,     /* A condition (cond 1) arised while picking a solution from the result obatined through transcendental equation - It need to be studied */
        ERR_TRANS_EQU_PICK_2    = -24,     /* A condition (cond 2) arised while picking a solution from the result obatined through transcendental equation - It need to be studied */
        ERR_TRANS_EQU_PICK_3    = -25,     /* A condition (cond 3) arised while picking a solution from the result obatined through transcendental equation - It need to be studied */
        ERR_TRANS_EQU_PICK_4    = -26,     /* A condition (cond 4) arised while picking a solution from the result obatined through transcendental equation - It need to be studied */
        ERR_TRANS_EQU_PICK_5    = -27,     /* A condition (cond 5) arised while picking a solution from the result obatined through transcendental equation - It need to be studied */
        ERR_PSI_REARRANGE_RANGE = -28,     /* Error while rearranging the psi range - It need to be studied */
        ERR_UNION_SINGLE        = -31,     /* Error in calculating the union of joints with only one feasible armangle */
        ERR_COMPLIMENT          = -32,     /* Error in calculating the complement of infeasible arm angle */
        ERR_UNION_ALL           = -33     /* There is no intersection in the feasible armangles */
        
    }error_code;
};

struct ArmAngle
{
        std::vector< std::pair<double,double> > psi;
        int joint_number;        
        std::string joint_name;
};

void Eul2RotMat(const double eul_zyx[3], std::vector<double> &rot_mat);
void quaternionToRotMat(const Eigen::Quaternion<double> &quat, std::vector<double> &rot_mat);
void Tra2Eul_pos(double tra[16], double *rot, double *pos);
void Mult_mat_mat(const std::vector<double> & mat1, const std::vector<double> &mat2, std::vector<double> &res);
void Mult_mat_vec(const std::vector<double> & mat, const std::vector<double> &vec, std::vector<double> &res);
void rot_matrix(const double &theta, const double &alpha, std::vector<double> &res);
void Mult_vec_tslvec(const std::vector<double> &vec, std::vector<double> &res);
void trans_mat(const std::vector<double> &mat, std::vector<double> &res);
void identityMatrix(std::vector<double> &dest);
void multMatrix(const std::vector<double> &src1, const std::vector<double> &src2, std::vector<double> &dest);
void multMatrix(double src1[16], double src2[16], double *dest);
int complement_of_infeasible_psi( const std::vector< ArmAngle > &infeasible_psi, std::vector< ArmAngle > &complimented_infeasbile_psi);
int union_joints_with_only_one_feasible_armangle(const std::vector< ArmAngle > &feasbile_armangle, std::vector< ArmAngle > &result);
int union_of_all_feasible_armangle(const std::vector< ArmAngle > &unsorted_feasible_psi, std::vector< std::pair<double,double>  > &final_feasible_armangle);
bool check_for_psi_range( const std::pair< double,double > psi_pair);


#endif
