#pragma once

#include "math.h"
#include<base/Eigen.hpp>
#include<base/JointsTrajectory.hpp>
#include<Eigen/Core>

#define PI 3.1415926535897932384626433832795
namespace kinematics_library{
void Eul2RotMat(double eul_zyx[3], double *rot_mat);
void Tra2Eul_pos(double tra[16], double *rot, double *pos);
void rotationMatrix2zyzEuler(double rotMat[9], double *zyzEuler);
double norm_vec(double vec[3]);
void scale_vec(double v[3], double factor, double *res);
void add_vec(double v1[3], double v2[3], double *res);
void cross_vec(double v1[3], double v2[3], double *res);
void extract_rot_Mat(double mat[16], double *res);
void build_transformation_mat(double rot_mat[9], double vec[3], double *res);
void copy_mat_4x4(double src[16], double *dest);;
void Mult_mat_vec(double mat[9], double vec[3], double *res);
void Mult_mat_mat_3x3(double mat1[9], double mat2[9], double *res);
void mult_mat_mat_4x4(double src1[16], double src2[16], double *dest);
void rot_matrix(double theta, double alpha, double *res);
void Mult_vec_tslvec(double vec[3], double *res);
void trans_mat(double mat[9], double *res);
void invert_mat(double mat[16], double *res);
void identityMatrix(double *dest);
double double_modulo(double divident, double divisor);

void rotationMatrix2zyxEuler(double *rotMat, double *zyxEuler);
void homogeneousMatrix2zyxEuler_pos(double *tra, double *zyxEuler,double* pos);
void homogeneousMatrix2rotationMatrix_pos(double *tra, double *rot_mat,double* pos);

void Array_2_EigenMat_4x4(double* src, Eigen::MatrixXd& dest);
void EigenMat4x4_2_Array(Eigen::MatrixXd& src, double* dest);
void EigenMat3x3_2_Array(base::MatrixXd& src, double* dest);


}