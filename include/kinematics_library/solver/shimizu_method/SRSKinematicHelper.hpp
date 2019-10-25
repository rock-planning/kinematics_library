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

#include "kinematics_library/abstract/KinematicsHelper.hpp"

struct SRSKinematic
{
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


int complement_of_infeasible_psi( const std::vector< ArmAngle > &infeasible_psi, std::vector< ArmAngle > &complimented_infeasbile_psi);
int union_joints_with_only_one_feasible_armangle(const std::vector< ArmAngle > &feasbile_armangle, std::vector< ArmAngle > &result);
int union_of_all_feasible_armangle(const std::vector< ArmAngle > &unsorted_feasible_psi, std::vector< std::pair<double,double>  > &final_feasible_armangle);
bool check_for_psi_range( const std::pair< double,double > psi_pair);


#endif
