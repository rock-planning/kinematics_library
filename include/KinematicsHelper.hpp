#ifndef _KINEMATICS_HELPER_HPP_
#define _KINEMATICS_HELPER_HPP_

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <base/Eigen.hpp>
#include <iostream>

namespace kinematics_library
{
    void zyxEuler2rotMat(const std::vector<double> &eul_zyx, Eigen::Matrix3d &rot_mat);
    void rotationMatrix2zyzEuler(const Eigen::Matrix3d &rotMat, double *zyz_euler);
    void quaternionToEuler(const base::Quaterniond &q, base::Vector3d &res);
    void eulerToQuaternion(const Eigen::Vector3d &eulerang, base::Quaterniond &res);
    void getPositionRotation(const Eigen::Matrix4d &hom_mat, base::Vector3d &fk_position, Eigen::Vector3d &fk_orientationZYX);
    void getPositionRotation(const Eigen::Matrix4d &homogeneous_matrix, base::Vector3d &fk_position, base::Quaterniond &fk_orientation);
    void rotationMatrix2zyxEuler(const double *rotMat, double *zyxEuler);
    void rotMat2QuaternionZYX(const double *rotMat, base::Quaterniond &orientationZYX);
    /**
    * @brief Get the position and orientation from the homogeneous matrix
    * @param fk_position position the homogeneous matrix
    * @param fk_orientationRPY orienation in roll, pitch, and yaw the homogeneous matrix
    */
    void quaternionToRotationMatrix(const base::Quaterniond &quat, Eigen::Matrix3d &rot_mat);
    void quaternionToRotationMatrixArray(const base::Quaterniond &quat, double *rot_mat);
    void getHomogeneousMatrix(const base::Vector3d &fk_position, const base::Quaterniond &fk_orientation, Eigen::Matrix4d &homogeneous_matrix);
    void inversematrix(const Eigen::Matrix4d &homogeneous_matrix, Eigen::Matrix4d &inverse_matrix);
    

}

#endif
