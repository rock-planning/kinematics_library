#include <abstract/KinematicsHelper.hpp>

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

void rotMat2QuaternionZYX(const double *rotMat, base::Quaterniond &orientationZYX)
{
    Eigen::Vector3d eulerang;

    rotationMatrix2zyxEuler(rotMat, eulerang);

    eulerToQuaternion(eulerang, orientationZYX);

}

void quaternionToEuler(const base::Quaterniond &q, base::Vector3d &res)
{
    res= Eigen::Vector3d::Zero();

    res(0) = atan2( (2.0*((q.w()+q.x())+(q.y()+q.z()))), (1.0-(2.0*((q.x()*q.x())+(q.y()*q.y()) ))) );
    res(1) = asin( 2.0*( (q.w() * q.y())-(q.z() * q.x()) ));
    res(2) = atan2( (2.0*((q.w()+q.z())+(q.x()+q.y()))), (1.0-(2.0*((q.y()*q.y())+(q.z()*q.z()) ))) );

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

void getPositionRotation(const Eigen::Matrix4d &hom_mat, base::Vector3d &fk_position, base::Quaterniond &fk_orientationZYX)
{
    // position
    fk_position(0) = hom_mat(0,3);
    fk_position(1) = hom_mat(1,3);
    fk_position(2) = hom_mat(2,3);

    //rotation
    base::Quaterniond quaternion_rot(hom_mat.topLeftCorner<3,3>());
    fk_orientationZYX = quaternion_rot;        
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
    Eigen::Matrix3d rot_matrix = fk_orientation.toRotationMatrix();

    matrix = Eigen::Matrix4d::Zero();
    matrix.topLeftCorner<3,3>() 	= rot_matrix;
    matrix.topRightCorner<3,1>() 	= fk_position;		
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
        LOG_FATAL("[KinematicsHelper]: Could not initiailise KDL transformation chain !!!!!!!");
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
        LOG_DEBUG("[KinematicsHelper]: Target basename = %s and kinematic basename = %s are not the same", 
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
        LOG_DEBUG("[RobotKinematics]: Target tipname = %s and kinematic tipname = %s are not the same", 
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
}
