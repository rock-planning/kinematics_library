#ifndef _KINEMATICS_HELPER_HPP_
#define _KINEMATICS_HELPER_HPP_

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <base/Eigen.hpp>
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>
#include <base-logging/Logging.hpp>

namespace kinematics_library
{
    static constexpr double PI              = 3.1415926535897932384626433832795;
    static constexpr double DTR             = 0.0174532925;    //Degree to Radians
    static constexpr double RTD             = 57.29577951;     //Radians to Degree    
    static constexpr double NAC             = -0.000005;
    static constexpr double PAC             = 0.000005;
    static constexpr double HALFDEGREE      = 0.008726646;
    static constexpr double EPSILON         = std::numeric_limits<double>::epsilon();
    static constexpr double ZERO_PRECISION  = 0.000001;
    
    void zyxEuler2rotMat(const std::vector<double> &eul_zyx, Eigen::Matrix3d &rot_mat);
    void rotationMatrix2zyzEuler(const Eigen::Matrix3d &rotMat, double *zyz_euler);
    void quaternionToEuler(const base::Quaterniond &q, base::Vector3d &res);
    void eulerToQuaternion(const Eigen::Vector3d &eulerang, base::Quaterniond &res);
    void getPositionRotation(const Eigen::Matrix4d &hom_mat, base::samples::RigidBodyState rbs_pose);
    void getPositionRotation(const Eigen::Matrix4d &hom_mat, base::Vector3d &fk_position, Eigen::Vector3d &fk_orientationZYX);
    void getPositionRotation(const Eigen::Matrix4d &homogeneous_matrix, base::Vector3d &fk_position, base::Quaterniond &fk_orientation);
    void rotationMatrix2zyxEuler(const double *&rotMat, Eigen::Vector3d &zyxEuler);
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
    void rbsToKdl(const base::samples::RigidBodyState &rbs, KDL::Frame &kdl);    
    void kdlToRbs(const KDL::Frame &kdl, base::samples::RigidBodyState &rbs);
    void transformFrame( const KDL::Tree &kdl_tree, const std::string &base_link, const std::string &tip_link, KDL::Frame &pose);
    bool transformFrame(const KDL::Tree &kdl_tree, const base::samples::Joints &joint_status, 
                        const std::string &base_frame, const std::string &tip_frame, KDL::Frame &fk_pose);
    void convertPoseBetweenDifferentFixedFrames( const KDL::Tree &kdl_tree, const base::samples::RigidBodyState &source_pose, 
                                            base::samples::RigidBodyState &target_pose);
    bool convertPoseBetweenDifferentFrames( const KDL::Tree &kdl_tree, const base::samples::Joints &joint_status, 
                                        const base::samples::RigidBodyState &source_pose, base::samples::RigidBodyState &target_pose);

    void convertVectorToKDLArray(const std::vector<double> &joint_angles, KDL::JntArray &kdl_jt_array);
    void convertKDLArrayToVector(const KDL::JntArray &kdl_jt_array, std::vector<double> &joint_angles);
    void convertKDLArrayToBaseJoints(const KDL::JntArray &kdl_jt_array, base::commands::Joints &joint_angles);
    void getKinematicJoints(const KDL::Chain &rev_jt_kdlchain, const base::samples::Joints &joint_angles, 
                            std::vector<std::string> &jt_names, std::vector<double> &kinematic_joints);
    void setKinematicJoints(const std::vector<double> &kinematic_joints, const std::vector<std::string> &jt_names, base::commands::Joints &joint_angles);
    
    
    // Needed for Shimizu method
    void eul2RotMat(const double eul_zyx[3], std::vector<double> &rot_mat);
    void quaternionToRotMat(const Eigen::Quaternion<double> &quat, std::vector<double> &rot_mat);
    void tra2EulPos(double tra[16], double *rot, double *pos);
    void multMatMat(const std::vector<double> & mat1, const std::vector<double> &mat2, std::vector<double> &res);
    void multMatMat(double src1[16], double src2[16], double *dest);
    void multMatVec(const std::vector<double> & mat, const std::vector<double> &vec, std::vector<double> &res);
    void rotMatrix(const double &theta, const double &alpha, std::vector<double> &res);
    void multVecTSLvec(const std::vector<double> &vec, std::vector<double> &res);
    void transMat(const std::vector<double> &mat, std::vector<double> &res);
    void identityMatrix(std::vector<double> &dest);
    void multMatrix(const std::vector<double> &src1, const std::vector<double> &src2, std::vector<double> &dest);

    // Needed for Asfour method
    double normVec(double vec[3]);
    void scaleVec(double v[3], double factor, double *res);
    void addVec(double v1[3], double v2[3], double *res);
    void crossVec(double v1[3], double v2[3], double *res);
    void extractRotMat(double mat[16], double *res);
    void buildTransformationMat(double rot_mat[9], double vec[3], double *res);
    void copyMat4x4(double src[16], double *dest);
    void invertMat(double mat[16], double *res);
    double doubleModulo(double divident, double divisor);
    void eigenMat3x3ToArray(base::MatrixXd& src, double* dest);
}

#endif
