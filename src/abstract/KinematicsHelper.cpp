#include "kinematics_library/abstract/KinematicsHelper.hpp"

namespace kinematics_library
{

void zyxEuler2rotMat(const std::vector<double> &eul_zyx, Eigen::Matrix3d &rot_mat)
{
    double ca=cos(eul_zyx.at(0)), sa=sin(eul_zyx.at(0));
    double cb=cos(eul_zyx.at(1)), sb=sin(eul_zyx.at(1));
    double cg=cos(eul_zyx.at(2)), sg=sin(eul_zyx.at(2));

    rot_mat(0,0) =  ca * cb; rot_mat(0,1) = (ca*sb*sg) - (sa*cg); rot_mat(0,2) = (ca*sb*cg) + (sa*sg);
    rot_mat(1,0) =  sa * cb; rot_mat(1,1) = (sa*sb*sg) + (ca*cg); rot_mat(1,2) = (sa*sb*cg) - (ca*sg);
    rot_mat(2,0) = -sb;      rot_mat(2,1) =  cb * sg;             rot_mat(2,2) =  cb * cg;

}

/**
 * 1st rotation around z-axis (alpha, zyzEuler[0])
 * 2nd rotation around new y-axis (beta, zyzEuler[1])
 * 3rd rotation around new z-axis (gamma, zyzEuler[2])
 *
 * Finds a solution for beta 0° <= beta <= 180° (another solution is possible by negating beta, and add/subtract pi to/from alpha and gamma)
 *
 * Scales -180° <= alpha/gamma <= 180°
 */
void rotationMatrix2zyzEuler(const Eigen::Matrix3d &rotMat, double *zyz_euler)
{
    double sin_beta = 0.0;

    zyz_euler[1] = atan2(sqrt((rotMat(2,0)*rotMat(2,0)) + (rotMat(2,1)*rotMat(2,1))), rotMat(2,2));

    //if(zyz_euler[1] < std::numeric_limits<double>::epsilon())
    //else if(zyz_euler[1] > M_PI)
    if( (zyz_euler[1] < std::numeric_limits<double>::epsilon()) &&
        (zyz_euler[1] > -std::numeric_limits<double>::epsilon()) )
    {
        zyz_euler[0] = 0.0;
        zyz_euler[1] = 0.0;
        zyz_euler[2] = atan2(-rotMat(0,1), rotMat(0,0));
    }
    else if((zyz_euler[1] > (M_PI - std::numeric_limits<double>::epsilon())) &&
            (zyz_euler[1] < (M_PI + std::numeric_limits<double>::epsilon())) )
    {
        zyz_euler[0] = 0.0;
        zyz_euler[1] = M_PI;
        zyz_euler[2] = atan2(rotMat(0,1), -rotMat(0,0));
    }
    else
    {
        sin_beta = sin(zyz_euler[1]);
        zyz_euler[0] = atan2(rotMat(1,2)/sin_beta,  rotMat(0,2)/sin_beta);
        zyz_euler[2] = atan2(rotMat(2,1)/sin_beta, -rotMat(2,0)/sin_beta);
    }
}

/**
 * 1st rotation around z-axis (gamma)
 * 2nd rotation around new y-axis (beta)
 * 3rd rotation around new x-axis (alpha)
 *
 * Finds a solution for beta -90° <= beta <= 90°
 *
 * Is equal to to a xyz-rotation around fixed angles
 *
 * @notation        The Matrix indices are as followed
 *                  0       1       2
 * M3x3 =           3       4       5       -->M3x3[9] = {0, 1, 2, 3, 4 , 5, 6, 7, 8}
 *                  6       7       8
 */

void rotationMatrix2zyxEuler(const double *&rotMat, Eigen::Vector3d &zyxEuler)
{
    zyxEuler(1) = atan2(-rotMat[6], sqrt(rotMat[0]*rotMat[0] + rotMat[3]*rotMat[3]));

    if(zyxEuler(1) < -M_PI/2.0+0.001){//ca. -PI/2
        zyxEuler(0) = 0.0;
        zyxEuler(1) = -M_PI/2.0;
        zyxEuler(2) = -atan2(rotMat[1], rotMat[4]);
    }
    else if(zyxEuler(1) > M_PI/2.0-0.001){//ca. PI/2
        zyxEuler(0) = 0.0;
        zyxEuler(1) = M_PI/2.0;
        zyxEuler(2) = atan2(rotMat[1], rotMat[4]);
    }
    else{
        zyxEuler(0) = atan2(rotMat[3]/cos(zyxEuler(1)), rotMat[0]/cos(zyxEuler(1)));
        zyxEuler(2) = atan2(rotMat[7]/cos(zyxEuler(1)), rotMat[8]/cos(zyxEuler(1)));
    }
}

void rotationMatrix2zyxEuler(const double *rotMat, double *zyxEuler)
{
    zyxEuler[1] = atan2(-rotMat[2], sqrt(rotMat[0]*rotMat[0] + rotMat[1]*rotMat[1]));

    if(zyxEuler[1] < -M_PI/2.0+0.001){//ca. -PI/2
        zyxEuler[0] = 0.0;
        zyxEuler[1] = -M_PI/2.0;
        zyxEuler[2] = -atan2(rotMat[3], rotMat[4]);
    }
    else if(zyxEuler[1] > M_PI/2.0-0.001){//ca. PI/2
        zyxEuler[0] = 0.0;
        zyxEuler[1] = M_PI/2.0;
        zyxEuler[2] = atan2(rotMat[3], rotMat[4]);
    }
    else
    {
        zyxEuler[0] = atan2(rotMat[1]/cos(zyxEuler[1]), rotMat[0]/cos(zyxEuler[1]));
        zyxEuler[2] = atan2(rotMat[5]/cos(zyxEuler[1]), rotMat[8]/cos(zyxEuler[1]));
    }
}

void rotMat2QuaternionZYX(const double *rotMat, base::Quaterniond &orientationZYX)
{
    Eigen::Vector3d eulerang;

    rotationMatrix2zyxEuler(rotMat, eulerang);

    eulerToQuaternion(eulerang, orientationZYX);

}

void quaternionToEuler(const base::Quaterniond &q, base::Vector3d &res)
{
    res= Eigen::Vector3d::Zero();
    
//     res(0) = atan2( (2.0*((q.w()+q.x())+(q.y()+q.z()))), (1.0-(2.0*((q.x()*q.x())+(q.y()*q.y()) ))) );
//     res(1) = asin( 2.0*( (q.w() * q.y())-(q.z() * q.x()) ));
//     res(2) = atan2( (2.0*((q.w()+q.z())+(q.x()+q.y()))), (1.0-(2.0*((q.y()*q.y())+(q.z()*q.z()) ))) );
    
    res = base::getEuler(q);

}

//converting eulerZYX to quaternion
void eulerToQuaternion(const Eigen::Vector3d &eulerang, base::Quaterniond &res)
{
    res = Eigen::Quaterniond::Identity();

    res.w() = (cos(eulerang(0)/2)*cos(eulerang(1)/2)*cos(eulerang(2)/2) ) + (sin(eulerang(0)/2)*sin(eulerang(1)/2)*sin(eulerang(2)/2) );
    res.z() = (sin(eulerang(0)/2)*cos(eulerang(1)/2)*cos(eulerang(2)/2) ) - (cos(eulerang(0)/2)*sin(eulerang(1)/2)*sin(eulerang(2)/2) );
    res.y() = (cos(eulerang(0)/2)*sin(eulerang(1)/2)*cos(eulerang(2)/2) ) + (sin(eulerang(0)/2)*cos(eulerang(1)/2)*sin(eulerang(2)/2) );
    res.x() = (cos(eulerang(0)/2)*cos(eulerang(1)/2)*sin(eulerang(2)/2) ) - (sin(eulerang(0)/2)*sin(eulerang(1)/2)*cos(eulerang(2)/2) );

    /*res.w() = (cos(eulerang(0)/2)*cos(eulerang(1)/2)*cos(eulerang(2)/2) ) + (sin(eulerang(0)/2)*sin(eulerang(1)/2)*sin(eulerang(2)/2) );
    res.y() = (cos(eulerang(0)/2)*cos(eulerang(1)/2)*sin(eulerang(2)/2) ) - (sin(eulerang(0)/2)*sin(eulerang(1)/2)*cos(eulerang(2)/2) );
    res.z() = (cos(eulerang(0)/2)*sin(eulerang(1)/2)*cos(eulerang(2)/2) ) + (sin(eulerang(0)/2)*cos(eulerang(1)/2)*sin(eulerang(2)/2) );
    res.x() = (sin(eulerang(0)/2)*cos(eulerang(1)/2)*cos(eulerang(0)/2) ) - (cos(eulerang(2)/2)*sin(eulerang(1)/2)*sin(eulerang(0)/2) );*/
}

void quaternionToRotationMatrix(const base::Quaterniond &quat, Eigen::Matrix3d &rot_mat)
{
    rot_mat(0,0) = 1-2*((quat.y()*quat.y()) +(quat.z()*quat.z()));  rot_mat(0,1) = 2*((quat.x()*quat.y()) - (quat.z()*quat.w()));   rot_mat(0,2) = 2*((quat.x()*quat.z()) + (quat.y()*quat.w()));
    rot_mat(1,0) = 2*((quat.x()*quat.y()) + (quat.z()*quat.w()));   rot_mat(1,1) = 1-2*((quat.x()*quat.x()) +(quat.z()*quat.z()));  rot_mat(1,2) = 2*((quat.y()*quat.z()) - (quat.x()*quat.w()));
    rot_mat(2,0) = 2*((quat.x()*quat.z()) - (quat.y()*quat.w()));   rot_mat(2,1) = 2*((quat.y()*quat.z()) + (quat.x()*quat.w()));   rot_mat(2,2) =  1-2*((quat.x()*quat.x()) +(quat.y()*quat.y()));


}

void getPositionRotation(const Eigen::Matrix4d &hom_mat, base::Vector3d &fk_position, Eigen::Vector3d &fk_orientationZYX)
{
    double cos_beta = 0.0;
    // position
    fk_position(0) = hom_mat(0,3);
    fk_position(1) = hom_mat(1,3);
    fk_position(2) = hom_mat(2,3);

    //orientation
    fk_orientationZYX(1) = atan2(-hom_mat(2,0), sqrt( (hom_mat(0,0)*hom_mat(0,0)) + (hom_mat(1,0)*hom_mat(1,0)) ));

    cos_beta = cos(fk_orientationZYX(1));

    if( (fk_orientationZYX(1) > (-M_PI/2.0 - std::numeric_limits<double>::epsilon())) &&
        (fk_orientationZYX(1) < (-M_PI/2.0 + std::numeric_limits<double>::epsilon())) )
    {
        //std::cout<<"im here  -90"<<std::endl;
        fk_orientationZYX(0) =  0.0;
        fk_orientationZYX(1) = -M_PI/2.0;
        fk_orientationZYX(2) = -atan2(hom_mat(0,1), hom_mat(1,1));
    }
    else if((fk_orientationZYX(1) > (M_PI/2.0 - std::numeric_limits<double>::epsilon())) &&
            (fk_orientationZYX(1) < (M_PI/2.0 + std::numeric_limits<double>::epsilon())) )
    {
        //std::cout<<"im here  90"<<std::endl;
        fk_orientationZYX(0) =  0.0;
        fk_orientationZYX(1) =  M_PI/2.0;
        fk_orientationZYX(2) =  atan2(hom_mat(0,1), hom_mat(1,1));
    }
    else
    {
        fk_orientationZYX(0) =  atan2(hom_mat(1,0)/cos_beta, hom_mat(0,0)/cos_beta);
        fk_orientationZYX(2) =  atan2(hom_mat(2,1)/cos_beta, hom_mat(2,2)/cos_beta);
    }
}

void getPositionRotation(const Eigen::Matrix4d &hom_mat, base::Vector3d &fk_position, base::Quaterniond &fk_orientation)
{
    // position
    fk_position(0) = hom_mat(0,3);
    fk_position(1) = hom_mat(1,3);
    fk_position(2) = hom_mat(2,3);

    //rotation    
    fk_orientation = base::Quaterniond(hom_mat.topLeftCorner<3,3>());
}

void getPositionRotation(const Eigen::Matrix4d &hom_mat, base::samples::RigidBodyState rbs_pose)
{
    getPositionRotation(hom_mat, rbs_pose.position, rbs_pose.orientation);
}

/*void getPositionRotation(const Eigen::Matrix4d &homogeneous_matrix, base::Vector3d &fk_position, base::Quaterniond &fk_orientation)
{
    Eigen::Vector3d fk_orientationZYX = Eigen::Vector3d::Identity();

    getPositionRotation(homogeneous_matrix, fk_position, fk_orientationZYX);

    eulerToQuaternion(fk_orientationZYX, fk_orientation);
}*/

void quaternionToRotationMatrixArray(const base::Quaterniond &quat, double *rot_mat)
{
    Eigen::Matrix3d rot_matrix;

    rot_matrix = quat.toRotationMatrix();

    rot_mat[0] = rot_matrix(0,0); rot_mat[1] = rot_matrix(0,1); rot_mat[2] = rot_matrix(0,2);
    rot_mat[3] = rot_matrix(1,0); rot_mat[4] = rot_matrix(1,1); rot_mat[5] = rot_matrix(1,2);
    rot_mat[6] = rot_matrix(2,0); rot_mat[7] = rot_matrix(2,1); rot_mat[8] = rot_matrix(2,2);

}

void getHomogeneousMatrix(const base::Vector3d &fk_position, const base::Quaterniond &fk_orientation, Eigen::Matrix4d &matrix)
{
    matrix = Eigen::Matrix4d::Zero();
    matrix.topLeftCorner<3,3>()  = fk_orientation.toRotationMatrix();
    matrix.topRightCorner<3,1>() = fk_position;
    matrix(3,3) = 1.0;		

}

void inversematrix(const Eigen::Matrix4d &homogeneous_matrix, Eigen::Matrix4d &inverse_matrix)
{
    inverse_matrix = Eigen::Matrix4d::Zero();

    inverse_matrix.topLeftCorner<3,3>() =   homogeneous_matrix.topLeftCorner<3,3>().transpose();
    inverse_matrix(0,3) = - homogeneous_matrix.topRightCorner<3,1>().dot(homogeneous_matrix.block<3,1>(0,0));
    inverse_matrix(1,3) = - homogeneous_matrix.topRightCorner<3,1>().dot(homogeneous_matrix.block<3,1>(0,1));
    inverse_matrix(2,3) = - homogeneous_matrix.topRightCorner<3,1>().dot(homogeneous_matrix.block<3,1>(0,2));
    inverse_matrix(3,3) = 1.0;

}

void convertVectorToKDLArray(const std::vector<double> &joint_angles, KDL::JntArray &kdl_jt_array)
{
    for(unsigned int i = 0; i <joint_angles.size(); i++  )
        kdl_jt_array.data(i) = joint_angles.at(i);

}

void convertKDLArrayToVector(const KDL::JntArray &kdl_jt_array, std::vector<double> &joint_angles)
{
    joint_angles.resize(kdl_jt_array.data.size());

    for(unsigned int i = 0; i <kdl_jt_array.data.size(); i++  )
        joint_angles.at(i) = kdl_jt_array.data(i);

}

void convertKDLArrayToBaseJoints(const KDL::JntArray &kdl_jt_array, base::commands::Joints &joint_angles)
{
    joint_angles.resize(kdl_jt_array.data.size());

    for(unsigned int i = 0; i <kdl_jt_array.data.size(); i++  )
        joint_angles.elements.at(i).position = kdl_jt_array.data(i);

}

void rbsToKdl(const base::samples::RigidBodyState &rbs, KDL::Frame &kdl)
{
    kdl.p.data[0] = rbs.position(0);
    kdl.p.data[1] = rbs.position(1);
    kdl.p.data[2] = rbs.position(2);

    kdl.M = KDL::Rotation::Quaternion( rbs.orientation.x(), rbs.orientation.y(),
                                       rbs.orientation.z(), rbs.orientation.w() );
}

void kdlToRbs(const KDL::Frame &kdl, base::samples::RigidBodyState &rbs)
{
    rbs.position(0) = kdl.p.data[0];
    rbs.position(1) = kdl.p.data[1];
    rbs.position(2) = kdl.p.data[2];

    kdl.M.GetQuaternion(rbs.orientation.x(), rbs.orientation.y(), rbs.orientation.z(), rbs.orientation.w());
}

void transformFrame( const KDL::Tree &kdl_tree, const std::string &base_link, const std::string &tip_link, KDL::Frame &pose)
{
    KDL::Chain new_chain;

    if(!kdl_tree.getChain(base_link , tip_link , new_chain))
    {
        LOG_FATAL("[KinematicsHelper]: Could not get KDL transformation chain between base_link: %s to tip_link: %s", base_link.c_str(), tip_link.c_str());
        exit(1);
    }
    else
    LOG_DEBUG("[KinematicsHelper]: KDL transformation chain initilised");

    for(std::size_t i=0; i<new_chain.segments.size(); i++ )
    {
        pose = pose * new_chain.getSegment(i).getFrameToTip();
    }
}

void convertPoseBetweenDifferentFrames(const KDL::Tree &kdl_tree, const base::samples::RigidBodyState &source_pose, base::samples::RigidBodyState &target_pose)
{
    //LOG_DEBUG_S<<"[KinematicsHelper]: IK function called for source frame "<<source_pose.sourceFrame.c_str()<<" and target frame "<<
    //                source_pose.targetFrame.c_str()<<" for the pose";
    //LOG_DEBUG("[KinematicsHelper]: Position:/n X: %f Y: %f Z: %f", source_pose.position(0), source_pose.position(1), source_pose.position(2));		
    //LOG_DEBUG("[KinematicsHelper]: Orientation:/n X: %f Y: %f Z: %f W: %f",
    //source_pose.orientation.x(), source_pose.orientation.y(), source_pose.orientation.z(), source_pose.orientation.w());

    target_pose.position = source_pose.position;
    target_pose.orientation = source_pose.orientation;

    if( (source_pose.sourceFrame.compare(target_pose.sourceFrame) != 0) && (!target_pose.sourceFrame.empty()) )
    {
        LOG_DEBUG("[KinematicsHelper]: Kinematic basename = %s and input basename = %s are not the same", 
        target_pose.sourceFrame.c_str(), source_pose.sourceFrame.c_str());

        // transform_base_tk_ -> transformation from target base to kinematic base
        KDL::Frame calculated_frame, new_frame;	    
        KDL::Frame transform_base_tk;
        transform_base_tk.Identity();	

        rbsToKdl(target_pose, calculated_frame);

        transformFrame( kdl_tree, target_pose.sourceFrame.c_str(), source_pose.sourceFrame.c_str(), transform_base_tk);
        
        //LOG_DEBUG_S<<"[KinematicsHelper]: Transformation betwwen target sourceframe to kinematic sourceframe ";
        //LOG_DEBUG("[KinematicsHelper]: Position:/n X: %f Y: %f Z: %f", transform_base_tk.p.x(), transform_base_tk.p.y(), transform_base_tk.p.z());          
        //LOG_DEBUG("[KinematicsHelper]: Orientation:/n X: %f Y: %f Z: %f W: %f",
        //source_pose.orientation.x(), source_pose.orientation.y(), source_pose.orientation.z(), source_pose.orientation.w());

        new_frame = transform_base_tk * calculated_frame;

        kdlToRbs(new_frame, target_pose);    

    }

    if( (source_pose.targetFrame.compare(target_pose.targetFrame) != 0) && (!target_pose.targetFrame.empty()) )
    {
        LOG_DEBUG("[RobotKinematics]: Kinematic tipname = %s and input tipname = %s are not the same", 
        target_pose.targetFrame.c_str(), source_pose.targetFrame.c_str());

        // transform_tip_kt_  -> transformation from kinematic tip to target tip
        KDL::Frame calculated_frame, new_frame;	    
        KDL::Frame transform_tip_kt;
        transform_tip_kt.Identity();

        transformFrame( kdl_tree, source_pose.targetFrame.c_str(), target_pose.targetFrame.c_str(), transform_tip_kt);
        
        //LOG_DEBUG_S<<"[KinematicsHelper]: Transformation betwwen target sourceframe to kinematic sourceframe ";
        //LOG_DEBUG("[KinematicsHelper]: Position:/n X: %f Y: %f Z: %f", transform_tip_kt.p.x(), transform_tip_kt.p.y(), transform_tip_kt.p.z());          
        
        rbsToKdl(target_pose, calculated_frame);
        new_frame = calculated_frame * transform_tip_kt;
        kdlToRbs(new_frame, target_pose);
    }

    if( (target_pose.sourceFrame.empty()) && target_pose.targetFrame.empty())
    {
        target_pose.sourceFrame = source_pose.sourceFrame;
        target_pose.targetFrame = source_pose.targetFrame;
    }

    //LOG_DEBUG_S<<"[RobotKinematics]: Target pose after frame transformation: source frame "<<target_pose.sourceFrame.c_str()<<"  target frame: "<<
    //                target_pose.targetFrame.c_str();
    //LOG_DEBUG("[RobotKinematics]: Position:/n X: %f Y: %f Z: %f", target_pose.position(0), target_pose.position(1), target_pose.position(2));		
    //LOG_DEBUG("[RobotKinematics]: Orientation:/n X: %f Y: %f Z: %f W: %f",
    //target_pose.orientation.x(), target_pose.orientation.y(), target_pose.orientation.z(), target_pose.orientation.w());

}

void getKinematicJoints(const KDL::Chain &rev_jt_kdlchain, const base::samples::Joints &joint_angles, 
                        std::vector<std::string> &jt_names, std::vector<double> &kinematic_joints)
{
    for(unsigned int i = 0; i < rev_jt_kdlchain.segments.size(); i++)
    {
        kinematic_joints.at(i) = joint_angles[rev_jt_kdlchain.getSegment(i).getJoint().getName()].position;	
        jt_names.at(i)         = rev_jt_kdlchain.getSegment(i).getJoint().getName();
    }
}

void setKinematicJoints(const std::vector<double> &kinematic_joints, const std::vector<std::string> &jt_names, base::commands::Joints &joint_angles)
{
    joint_angles = base::samples::Joints::Positions(kinematic_joints, jt_names);
    for (std::size_t i = 0; i != joint_angles.elements.size(); ++i)
    {
        joint_angles[i].speed = 0.0;
        joint_angles[i].effort = 0.0;
    }
}


void eul2RotMat(const double eul_zyx[3], std::vector<double> &rot_mat)
{
    double ca=cos(eul_zyx[0]), sa=sin(eul_zyx[0]), cb=cos(eul_zyx[1]), sb=sin(eul_zyx[1]), cg=cos(eul_zyx[2]), sg=sin(eul_zyx[2]);

    rot_mat.at(0)= ca*cb;   rot_mat.at(3)=(ca*sb*sg)-(sa*cg);     rot_mat.at(6)=(ca*sb*cg)+(sa*sg);
    rot_mat.at(1)= sa*cb;   rot_mat.at(4)=(sa*sb*sg)+(ca*cg);     rot_mat.at(7)=(sa*sb*cg)-(ca*sg);
    rot_mat.at(2)=-sb;      rot_mat.at(5)=cb*sg;                  rot_mat.at(8)=cb*cg;
}

void quaternionToRotMat(const Eigen::Quaternion<double> &quat, std::vector<double> &rot_mat)
{
//     rot_mat.at(0) = 1-2*((quat.y()*quat.y()) +(quat.z()*quat.z()));  rot_mat.at(3) = 2*((quat.x()*quat.y()) - (quat.z()*quat.w()));   rot_mat.at(6) = 2*((quat.x()*quat.z()) + (quat.y()*quat.w()));
//     rot_mat.at(1) = 2*((quat.x()*quat.y()) + (quat.z()*quat.w()));   rot_mat.at(4) = 1-2*((quat.x()*quat.x()) +(quat.z()*quat.z()));  rot_mat.at(7) = 2*((quat.y()*quat.z()) - (quat.x()*quat.w()));
//     rot_mat.at(2) = 2*((quat.x()*quat.z()) - (quat.y()*quat.w()));   rot_mat.at(5) = 2*((quat.y()*quat.z()) + (quat.x()*quat.w()));   rot_mat.at(8) =  1-2*((quat.x()*quat.x()) +(quat.y()*quat.y()));

    //use of eigen library
//     Eigen::Matrix3d quat_rot_mat = quat.normalized().toRotationMatrix();
//     for(int i = 0; i < 3; i++)
//     {
//         for(int j = 0; j < 3; j++)
//             rot_mat.at(i+j) = quat_rot_mat(j,i);
//     }
    
    Eigen::Matrix3d mat = quat.toRotationMatrix();
    rot_mat.at(0) = mat(0,0); rot_mat.at(3) = mat(0,1); rot_mat.at(6) = mat(0,2);
    rot_mat.at(1) = mat(1,0); rot_mat.at(4) = mat(1,1); rot_mat.at(7) = mat(1,2);
    rot_mat.at(2) = mat(2,0); rot_mat.at(5) = mat(2,1); rot_mat.at(8) = mat(2,2);
    
}

void tra2EulPos(double tra[16], double *rot, double *pos)
{

    pos[0]=tra[12];
    pos[1]=tra[13];
    pos[2]=tra[14];

    rot[1]=atan2(-tra[2],sqrt((tra[0]*tra[0])+(tra[1]*tra[1])));
    rot[0]=atan2((tra[1]/(cos(rot[1]))),(tra[0]/cos(rot[1])));
    rot[2]=atan2((tra[6]/(cos(rot[1]))),(tra[10]/cos(rot[1])));
}

void multMatMat(const std::vector<double> &mat1, const std::vector<double> & mat2, std::vector<double> &res)
{
    res.at(0)=(mat1.at(0)*mat2.at(0))+(mat1.at(3)*mat2.at(1))+(mat1.at(6)*mat2.at(2));    res.at(3)=(mat1.at(0)*mat2.at(3))+(mat1.at(3)*mat2.at(4))+(mat1.at(6)*mat2.at(5));    res.at(6)=(mat1.at(0)*mat2.at(6))+(mat1.at(3)*mat2.at(7))+(mat1.at(6)*mat2.at(8));
    res.at(1)=(mat1.at(1)*mat2.at(0))+(mat1.at(4)*mat2.at(1))+(mat1.at(7)*mat2.at(2));    res.at(4)=(mat1.at(1)*mat2.at(3))+(mat1.at(4)*mat2.at(4))+(mat1.at(7)*mat2.at(5));    res.at(7)=(mat1.at(1)*mat2.at(6))+(mat1.at(4)*mat2.at(7))+(mat1.at(7)*mat2.at(8));
    res.at(2)=(mat1.at(2)*mat2.at(0))+(mat1.at(5)*mat2.at(1))+(mat1.at(8)*mat2.at(2));    res.at(5)=(mat1.at(2)*mat2.at(3))+(mat1.at(5)*mat2.at(4))+(mat1.at(8)*mat2.at(5));    res.at(8)=(mat1.at(2)*mat2.at(6))+(mat1.at(5)*mat2.at(7))+(mat1.at(8)*mat2.at(8));


}

void multMatVec(const std::vector<double> &mat, const std::vector<double> &vec, std::vector<double> &res)
{
    res.at(0)=(mat.at(0)*vec.at(0))+(mat.at(3)*vec.at(1))+(mat.at(6)*vec.at(2));
    res.at(1)=(mat.at(1)*vec.at(0))+(mat.at(4)*vec.at(1))+(mat.at(7)*vec.at(2));
    res.at(2)=(mat.at(2)*vec.at(0))+(mat.at(5)*vec.at(1))+(mat.at(8)*vec.at(2));
}

void rotMatrix(const double &theta, const double &alpha, std::vector<double> &res)
{
    double ca=cos(alpha), sa=sin(alpha), ct=cos(theta), st=sin(theta);

    if (fabs(alpha) == kinematics_library::PI/2.0)
            ca = 0.0;

    for(int i = 0; i < 9; i++)
            res.at(i) = 0.0;

    res.at(0) = ct;  res.at(3) = -st*ca;    res.at(6) =  st*sa;
    res.at(1) = st;  res.at(4) =  ct*ca;    res.at(7) = -ct*sa;
    res.at(2) = 0.0;   res.at(5) =  sa;       res.at(8) =  ca;

    /*res.at(0) = ct;  res.at(1) = st*ca;    res.at(2) =  st*sa;
    res.at(3) = -st;  res.at(4) =  ct*ca;    res.at(5) = ct*sa;
    res.at(6) = 0.0;   res.at(7) =  -sa;       res.at(8) =  ca;*/
}

void multVecTSLvec(const std::vector<double> &vec, std::vector<double> &res)
{
    res.at(0)=vec.at(0)*vec.at(0);  res.at(3)=vec.at(0)*vec.at(1);   res.at(6)=vec.at(0)*vec.at(2);
    res.at(1)=vec.at(1)*vec.at(0);  res.at(4)=vec.at(1)*vec.at(1);   res.at(7)=vec.at(1)*vec.at(2);
    res.at(2)=vec.at(2)*vec.at(0);  res.at(5)=vec.at(2)*vec.at(1);   res.at(8)=vec.at(2)*vec.at(2);
}

void transMat(const std::vector<double> &mat, std::vector<double> &res)
{
    res.at(0)=mat.at(0);  res.at(3)=mat.at(1);   res.at(6)=mat.at(2);
    res.at(1)=mat.at(3);  res.at(4)=mat.at(4);   res.at(7)=mat.at(5);
    res.at(2)=mat.at(6);  res.at(5)=mat.at(7);   res.at(8)=mat.at(8);
}

void identityMatrix(std::vector<double> &dest)
{
    for(int i = 0; i < 16; i++)
                dest.at(i) = 0.0;

        dest.at(0) = 1.0;
        dest.at(5) = 1.0;
        dest.at(10) = 1.0;
        dest.at(15) = 1.0;
}

void multMatrix(const std::vector<double> &src1, const std::vector<double> &src2, std::vector<double> &dest)
{
    dest.at(0)  = src1.at(0)*src2.at(0) + src1.at(4)*src2.at(1) + src1.at(8)*src2.at(2)   + src1[12]*src2.at(3);
    dest.at(1)  = src1.at(1)*src2.at(0) + src1.at(5)*src2.at(1) + src1.at(9)*src2.at(2)   + src1[13]*src2.at(3);
    dest.at(2)  = src1.at(2)*src2.at(0) + src1.at(6)*src2.at(1) + src1.at(10)*src2.at(2)  + src1[14]*src2.at(3);
    dest.at(3)  = src1.at(3)*src2.at(0) + src1.at(7)*src2.at(1) + src1.at(11)*src2.at(2)  + src1[15]*src2.at(3);
    dest.at(4)  = src1.at(0)*src2.at(4) + src1.at(4)*src2.at(5) + src1.at(8)*src2.at(6)   + src1[12]*src2.at(7);
    dest.at(5)  = src1.at(1)*src2.at(4) + src1.at(5)*src2.at(5) + src1.at(9)*src2.at(6)   + src1[13]*src2.at(7);
    dest.at(6)  = src1.at(2)*src2.at(4) + src1.at(6)*src2.at(5) + src1.at(10)*src2.at(6)  + src1[14]*src2.at(7);
    dest.at(7)  = src1.at(3)*src2.at(4) + src1.at(7)*src2.at(5) + src1.at(11)*src2.at(6)  + src1[15]*src2.at(7);
    dest.at(8)  = src1.at(0)*src2.at(8) + src1.at(4)*src2.at(9) + src1.at(8)*src2.at(10)  + src1[12]*src2.at(11);
    dest.at(9)  = src1.at(1)*src2.at(8) + src1.at(5)*src2.at(9) + src1.at(9)*src2.at(10)  + src1[13]*src2.at(11);
    dest.at(10) = src1.at(2)*src2.at(8) + src1.at(6)*src2.at(9) + src1.at(10)*src2.at(10) + src1[14]*src2.at(11);
    dest.at(11) = src1.at(3)*src2.at(8) + src1.at(7)*src2.at(9) + src1.at(11)*src2.at(10) + src1[15]*src2.at(11);
    dest[12]    = src1.at(0)*src2[12]   + src1.at(4)*src2[13]   + src1.at(8)*src2[14]     + src1[12]*src2[15];
    dest[13]    = src1.at(1)*src2[12]   + src1.at(5)*src2[13]   + src1.at(9)*src2[14]     + src1[13]*src2[15];
    dest[14]    = src1.at(2)*src2[12]   + src1.at(6)*src2[13]   + src1.at(10)*src2[14]    + src1[14]*src2[15];
    dest[15]    = src1.at(3)*src2[12]   + src1.at(7)*src2[13]   + src1.at(11)*src2[14]    + src1[15]*src2[15];
}

void multMatMat(double src1[16], double src2[16], double *dest)
{
    dest[0]  = src1[0]*src2[0]  + src1[4]*src2[1]  + src1[8]*src2[2]   + src1[12]*src2[3];
    dest[1]  = src1[1]*src2[0]  + src1[5]*src2[1]  + src1[9]*src2[2]   + src1[13]*src2[3];
    dest[2]  = src1[2]*src2[0]  + src1[6]*src2[1]  + src1[10]*src2[2]  + src1[14]*src2[3];
    dest[3]  = src1[3]*src2[0]  + src1[7]*src2[1]  + src1[11]*src2[2]  + src1[15]*src2[3];
    dest[4]  = src1[0]*src2[4]  + src1[4]*src2[5]  + src1[8]*src2[6]   + src1[12]*src2[7];
    dest[5]  = src1[1]*src2[4]  + src1[5]*src2[5]  + src1[9]*src2[6]   + src1[13]*src2[7];
    dest[6]  = src1[2]*src2[4]  + src1[6]*src2[5]  + src1[10]*src2[6]  + src1[14]*src2[7];
    dest[7]  = src1[3]*src2[4]  + src1[7]*src2[5]  + src1[11]*src2[6]  + src1[15]*src2[7];
    dest[8]  = src1[0]*src2[8]  + src1[4]*src2[9]  + src1[8]*src2[10]  + src1[12]*src2[11];
    dest[9]  = src1[1]*src2[8]  + src1[5]*src2[9]  + src1[9]*src2[10]  + src1[13]*src2[11];
    dest[10] = src1[2]*src2[8]  + src1[6]*src2[9]  + src1[10]*src2[10] + src1[14]*src2[11];
    dest[11] = src1[3]*src2[8]  + src1[7]*src2[9]  + src1[11]*src2[10] + src1[15]*src2[11];
    dest[12] = src1[0]*src2[12] + src1[4]*src2[13] + src1[8]*src2[14]  + src1[12]*src2[15];
    dest[13] = src1[1]*src2[12] + src1[5]*src2[13] + src1[9]*src2[14]  + src1[13]*src2[15];
    dest[14] = src1[2]*src2[12] + src1[6]*src2[13] + src1[10]*src2[14] + src1[14]*src2[15];
    dest[15] = src1[3]*src2[12] + src1[7]*src2[13] + src1[11]*src2[14] + src1[15]*src2[15];
}

double normVec(double vec[3])
{
    double result = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
    return result;
}

void scaleVec(double v[3], double factor, double *res)
{
    for(int i = 0; i < 3; i++)
        res[i] = v[i] * factor;
}

void addVec(double v1[3], double v2[3], double *res)
{
    for(int i = 0; i < 3; i++)
        res[i] = v1[i] + v2[i];
}

void crossVec(double v1[3], double v2[3], double *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void extractRotMat(double mat[16], double *res)
{
    res[0] = mat[0];
    res[1] = mat[1];
    res[2] = mat[2];
    res[3] = mat[4];
    res[4] = mat[5];
    res[5] = mat[6];
    res[6] = mat[8];
    res[7] = mat[9];
    res[8] = mat[10];
}

void buildTransformationMat(double rot_mat[9], double vec[3], double *res)
{
    res[0]=rot_mat[0];  res[4]=rot_mat[3];   res[8]=rot_mat[6];   res[12]=vec[0];
    res[1]=rot_mat[1];  res[5]=rot_mat[4];   res[9]=rot_mat[7];   res[13]=vec[1];
    res[2]=rot_mat[2];  res[6]=rot_mat[5];   res[10]=rot_mat[8];  res[14]=vec[2];
    res[3]=0.0;         res[7]=0.0;          res[11]=0.0;         res[15]= 1.0;
}


void copyMat4x4(double src[16], double *dest)
{
    for(int j = 0; j < 16; j++)
        dest[j] = src[j];
}

void multMatVec(double mat[9], double vec[3], double *res)
{
  res[0]=(mat[0]*vec[0])+(mat[3]*vec[1])+(mat[6]*vec[2]);
  res[1]=(mat[1]*vec[0])+(mat[4]*vec[1])+(mat[7]*vec[2]);
  res[2]=(mat[2]*vec[0])+(mat[5]*vec[1])+(mat[8]*vec[2]);
}


void transMat(double mat[9], double *res)
{
  res[0]=mat[0];  res[3]=mat[1];   res[6]=mat[2];
  res[1]=mat[3];  res[4]=mat[4];   res[7]=mat[5];
  res[2]=mat[6];  res[5]=mat[7];   res[8]=mat[8];
}

void invertMat(double mat[16], double *res)
{
  double rot_mat[9];
  double t_mat[9];
  double pos_vec[3] = {mat[12], mat[13], mat[14]};
  double new_pos[3];
  double new_pos_neg[3];

  extractRotMat(mat, rot_mat);
  transMat(rot_mat, t_mat);
  multMatVec(t_mat, pos_vec, new_pos);
  scaleVec(new_pos, -1, new_pos_neg);

  res[0]=t_mat[0];  res[4]=t_mat[3];   res[8]=t_mat[6];   res[12]=new_pos_neg[0];
  res[1]=t_mat[1];  res[5]=t_mat[4];   res[9]=t_mat[7];   res[13]=new_pos_neg[1];
  res[2]=t_mat[2];  res[6]=t_mat[5];   res[10]=t_mat[8];  res[14]=new_pos_neg[2];
  res[3]=0.0;       res[7]=0.0;        res[11]=0.0;       res[15]=1.0;
}


double doubleModulo(double divident, double divisor){
  // double d_result = divident / divisor;
  // printf("divident %f / divisor %f = %f\n", divident, divisor, d_result);
  int int_result = (int) ( divident / divisor );
  // printf("int_result: %d\n", int_result);
  double result = (divident - (double) int_result * divisor);
  // printf("%f transforms to %f\n", divident, result);
  return result;
}


void eigenMat3x3ToArray(base::MatrixXd& src, double* dest)
{
    dest[0] = src(0,0);     dest[3] = src(0,1);     dest[6] = src(0,2);     
    dest[1] = src(1,0);     dest[4] = src(1,1);     dest[7] = src(1,2);    
    dest[2] = src(2,0);     dest[5] = src(2,1);     dest[8] = src(2,2); 
}
}
