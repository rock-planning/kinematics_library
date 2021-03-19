#include "kinematics_library/HandleKinematicConfig.hpp"

namespace kinematics_library
{

SRSKinematicSolver::SRSKinematicSolver ( const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree,                                          
                                         const KDL::Chain &kdl_chain ): 
                                         jts_limits_(jts_limits)
{
    kdl_tree_  = kdl_tree;
    kdl_chain_ = kdl_chain;
//     jts_limits_[0].first = -90*kinematics_library::DTR; jts_limits_[0].second = 90*kinematics_library::DTR;
//     jts_limits_[1].first = -45*kinematics_library::DTR; jts_limits_[1].second = 45*kinematics_library::DTR;
//     jts_limits_[2].first = -120*kinematics_library::DTR; jts_limits_[2].second = 120*kinematics_library::DTR;
//     jts_limits_[3].first =  0*kinematics_library::DTR; jts_limits_[3].second = 135*kinematics_library::DTR;
//     jts_limits_[4].first = -90*kinematics_library::DTR; jts_limits_[4].second = 90*kinematics_library::DTR;
//     jts_limits_[5].first = -90*kinematics_library::DTR; jts_limits_[5].second = 90*kinematics_library::DTR;
//     jts_limits_[6].first = -120*kinematics_library::DTR; jts_limits_[6].second = 120*kinematics_library::DTR;
}

SRSKinematicSolver::~SRSKinematicSolver()
{}

bool SRSKinematicSolver::loadKinematicConfig( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status)
{    
    // assign the config
    YAML::Node input_config;
    // check whether the config could be loaded or not.
    if(!handle_kinematic_config::loadConfigFile(kinematics_config.solver_config_abs_path, kinematics_config.solver_config_filename, input_config))
    {
        LOG_ERROR("[SRSKinematicSolver]: Unable to load kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(),
                     kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::NO_CONFIG_FILE;
        return false;
    }
    
    const YAML::Node& srs_config_node = input_config["srs_config"];
    if(!handle_kinematic_config::getSRSConfig(srs_config_node, srs_config_))
    {
        LOG_ERROR("[SRSKinematicSolver]: Unable to read kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(),
                     kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::CONFIG_READ_ERROR;
        return false;
    }

    assignVariables ( kinematics_config, kdl_chain_ );
    kdl_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    kdl_ik_jt_array_.resize ( kdl_chain_.getNrOfJoints() );

    // base-shoulder vector
    l_bs.resize(3, 0.0);
    l_bs.at(0) = 0;
    l_bs.at(1) = 0;
    l_bs.at(2) = srs_config_.offset_base_shoulder;

    // shoulder-elbow vector
    l_se.resize(3, 0.0);
    l_se.at(0) = 0;
    l_se.at(1) = -srs_config_.offset_shoulder_elbow;
    l_se.at(2) = 0;

    // elbow-wrist vector
    l_ew.resize(3, 0.0);
    l_ew.at(0) = 0;
    l_ew.at(1) = 0.0;
    l_ew.at(2) = srs_config_.offset_elbow_wrist;

    // wrist-tool vector
    l_wt.resize(3, 0.0);
    l_wt.at(0) = 0;
    l_wt.at(1) = 0;
    l_wt.at(2) = srs_config_.offset_wrist_tool;

    return true;

}

bool SRSKinematicSolver::solveIK (const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                            std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status )
{
    convertPoseBetweenDifferentFrames ( kdl_tree_, target_pose, kinematic_pose_ );
    
    getKinematicJoints ( kdl_chain_, joint_status, jt_names_, current_jt_status_ );
    
    // currently we will use only one solution. 
    // The solver could give "all" the possible solutions by incrementing the arm angle, thats the beauty of this solver
    solution.resize(1);
    solution[0].names = jt_names_;
    solution[0].elements.resize(jt_names_.size());
    
    int res = invkin(target_pose.position, target_pose.orientation, solution[0]);
    
//     base::Position pos_test;    
//     Eigen::Matrix3d rot_mat;
    
//     pos_test(0) = 0.5; pos_test(1) = 0.2; pos_test(2) = 0.7;
//     
//     rot_mat(0,0) = 0.067; rot_mat(0,1) = 0.933; rot_mat(0,2) = 0.354;
//     rot_mat(1,0) = 0.932; rot_mat(1,1) = 0.067; rot_mat(1,2) =-0.354;
//     rot_mat(2,0) =-0.354; rot_mat(2,1) = 0.354; rot_mat(2,2) =-0.866;
//     
//     pos_test(0) = 0.65; pos_test(1) = 0.0; pos_test(2) = 0.5;
//     
//     rot_mat(0,0) = 0.0; rot_mat(0,1) =-1.0; rot_mat(0,2) = 0.0;
//     rot_mat(1,0) =-1.0; rot_mat(1,1) = 0.0; rot_mat(1,2) = 0.0;
//     rot_mat(2,0) = 0.0; rot_mat(2,1) = 0.0; rot_mat(2,2) =-1.0;
//     
//     Eigen::Quaternion<double> g;
//     g= rot_mat;
//     
//     int res = invkin(pos_test, g, solution[0]);
//         
    if( res == SRSKinematic::SUCCESS)
    {
        solver_status.statuscode = KinematicsStatus::IK_FOUND;
        return true;
    }    
    else
    {
        printError(res);
        solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
        return false;
    }
    
}

bool SRSKinematicSolver::solveFK (const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status )
{
    getKinematicJoints ( kdl_chain_, joint_angles, jt_names_, current_jt_status_ );

    convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );

//     if ( fk_kdlsolver_pos_->JntToCart ( kdl_jt_array_, kdl_frame_ ) >= 0 ) 
//     {
//         kdlToRbs ( kdl_frame_, kinematic_pose_ );
        solver_status.statuscode = KinematicsStatus::FK_FOUND;
//         convertPoseBetweenDifferentFrames ( kdl_tree_, kinematic_pose_, fk_pose );
        fk_pose = SRSKinematicSolver::direct(joint_angles);
        return true;
//     }

    solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
    return false;
}


base::samples::RigidBodyState SRSKinematicSolver::direct(const base::samples::Joints &joint_angles)
{
   double fk[7][16];//variable for forward kinemtaics 
   double sa[7], ca[7];                            // sine and cosine array used on forward kinematics
   double lk_ofst[7];                              // link offsets
   
   // initialising the link offsets
    lk_ofst[0] = srs_config_.offset_base_shoulder;
    lk_ofst[1] = 0;
    lk_ofst[2] = srs_config_.offset_shoulder_elbow;
    lk_ofst[3] = 0;
    lk_ofst[4] = srs_config_.offset_elbow_wrist;
    lk_ofst[5] = 0;
    lk_ofst[6] = srs_config_.offset_wrist_tool;

    // initialising the sine and cosine array used in forward kinematics
    sa[0] = sin(-kinematics_library::PI/2.0);
    sa[1] = sin( kinematics_library::PI/2.0);
    sa[2] = sin(-kinematics_library::PI/2.0);
    sa[3] = sin( kinematics_library::PI/2.0);
    sa[4] = sin(-kinematics_library::PI/2.0);
    sa[5] = sin( kinematics_library::PI/2.0);
    sa[6] = sin(0.0);
    ca[0] = cos(-kinematics_library::PI/2.0);
    ca[1] = cos( kinematics_library::PI/2.0);
    ca[2] = cos(-kinematics_library::PI/2.0);
    ca[3] = cos( kinematics_library::PI/2.0);
    ca[4] = cos(-kinematics_library::PI/2.0);
    ca[5] = cos( kinematics_library::PI/2.0);
    ca[6] = cos(0.0);
    
    
    double ct=0.0, st=0.0;
    double tmpMat[16] = {   1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f  };

    ct = cos(joint_angles.elements[0].position);
    st = sin(joint_angles.elements[0].position);

    // calculate the joint matrix
    fk[0][0] = ct;  fk[0][4] = -st* ca[0];  fk[0][8] = st * sa[0];  fk[0][12] = 0;
    fk[0][1] = st;  fk[0][5] =  ct * ca[0]; fk[0][9] =-ct * sa[0];  fk[0][13] = 0;
    fk[0][2] = 0;   fk[0][6] =  sa[0];      fk[0][10]= ca[0];       fk[0][14] = lk_ofst[0];
    fk[0][3] = 0;   fk[0][7] =  0;          fk[0][11]= 0;           fk[0][15] = 1;

    for(int i = 1; i < 7; i++)
    {
        ct = cos(joint_angles.elements[i].position);
        st = sin(joint_angles.elements[i].position);

        // calculate the joint matrix
        tmpMat[0] = ct;         tmpMat[4] = -st * ca[i];        tmpMat[8] = st * sa[i];
        tmpMat[1] = st;         tmpMat[5] =  ct * ca[i];        tmpMat[9] =-ct * sa[i];
                                tmpMat[6] = sa[i];              tmpMat[10]= ca[i];              tmpMat[14] = lk_ofst[i];

        // accumulate it
        multMatMat(fk[i-1], tmpMat, fk[i]);
    }
    
    
    base::samples::RigidBodyState fk_pose;
       
        
    Eigen::Matrix3d rot_mat;

    rot_mat(0,0) = fk[6][0]; rot_mat(0,1) = fk[6][4]; rot_mat(0,2) = fk[6][8];
    rot_mat(1,0) = fk[6][1]; rot_mat(1,1) = fk[6][5]; rot_mat(1,2) = fk[6][9];
    rot_mat(2,0) = fk[6][2]; rot_mat(2,1) = fk[6][6]; rot_mat(2,2) = fk[6][10];    
    
    fk_pose.orientation = rot_mat;
    
    fk_pose.position(0) = fk[6][12];fk_pose.position(1) = fk[6][13];fk_pose.position(2) = fk[6][14];
    
    return fk_pose;
    

}


int SRSKinematicSolver::invkin(const base::Position &pos, const base::Quaterniond &rot, base::commands::Joints &jointangles)
{
    int succeeded = 0;          // output

    double m_Xsw=0.0;           // length vector: shoulder to wrist

    double c1=0.0, s1=0.0, c2=0.0, s2=0.0, the1_R_var1=0.0;
    double the1_R=0.0, the2_R=0.0;

    std::vector<double> Xsw(3, 0.0);              // vector: shoulder to wrist

    std::vector<double> t_refthe2(3, 0.0); // vector related to temporal reference theta 2
    std::vector<double> usw(3, 0.0);
    std::vector<double> t_R34(3, 0.0);
    std::vector<double> t_Xsw(3, 0.0);
    std::vector<double> usw_sk(9, 0.0);
    std::vector<double> As(9, 0.0), Bs(9, 0.0), Cs(9, 0.0), Aw(9, 0.0), Bw(9, 0.0), Cw(9, 0.0);
    std::vector<double> t_Bs(9, 0.0), t_Cs(9, 0.0), t_Aw(9, 0.0), t_Bw(9, 0.0), t_Cw(9, 0.0);
    std::vector<double> tsl_R34(9, 0.0), tsl_As(9, 0.0), tsl_Bs(9, 0.0), tsl_Cs(9, 0.0);
    std::vector<double> t_r03_r(9, 0.0);
    std::vector<double> R01(9, 0.0), R12(9, 0.0), R23(9, 0.0), R_03_R(9, 0.0), R34(9, 0.0);
    std::vector<double> Rd(9, 0.0);

    double t_thet4,thet4;
    int armAngle_result = 0;
    double AA = 0.0;
    std::vector< std::pair<double,double>  > final_feasible_armangle;

    //Eul2RotMat(rot,Rd);  //eul_zyx
    quaternionToRotMat(rot,Rd);
    multMatVec(Rd,l_wt,t_Xsw);
    std::cout<<"Pos = "<<pos<<std::endl;
    Xsw.at(0) = pos(0)-l_bs.at(0)-t_Xsw.at(0);
    Xsw.at(1) = pos(1)-l_bs.at(1)-t_Xsw.at(1);
    Xsw.at(2) = pos(2)-l_bs.at(2)-t_Xsw.at(2);
    
    std::cout<<"l_bs = "<<l_bs.at(0)<<"  "<<l_bs.at(1)<<"  "<<l_bs.at(2)<<std::endl;
    std::cout<<"t_x = "<<t_Xsw.at(0)<<"  "<<t_Xsw.at(1)<<"  "<<t_Xsw.at(2)<<std::endl;
    std::cout<<pos(0)-l_bs.at(0)-t_Xsw.at(0)<<std::endl;
    std::cout<<pos(1)-l_bs.at(1)-t_Xsw.at(1)<<std::endl;
    std::cout<<pos(2)-l_bs.at(2)-t_Xsw.at(2)<<std::endl;

    t_thet4=((Xsw.at(0)*Xsw.at(0))+(Xsw.at(1)*Xsw.at(1))+(Xsw.at(2)*Xsw.at(2)));

    if (sqrt(t_thet4) >(srs_config_.offset_shoulder_elbow + srs_config_.offset_elbow_wrist))
    {
        succeeded = SRSKinematic::ERR_REACH;
    }
    else
    {
        thet4=((t_thet4 - (srs_config_.offset_shoulder_elbow * srs_config_.offset_shoulder_elbow) 
                        - (srs_config_.offset_elbow_wrist * srs_config_.offset_elbow_wrist)) / (2.0 * srs_config_.offset_shoulder_elbow * srs_config_.offset_elbow_wrist) );

        jointangles.elements.at(3).position = acos(thet4);  //using cosine law


        std::cout<< "JOINT 4 = "<<jointangles.elements.at(3).position*kinematics_library::RTD<<"   "<<t_thet4<<"  "<<sqrt(1-(thet4*thet4))<<std::endl;

        //Below calculate the reference shoulder angle
        rotMatrix(jointangles.elements.at(3).position, kinematics_library::PI /2.0, R34);

        multMatVec(R34, l_ew, t_R34);

        t_refthe2.at(0) = l_se.at(0) + t_R34.at(0);
        t_refthe2.at(1) = l_se.at(1) + t_R34.at(1);
        t_refthe2.at(2) = l_se.at(2) + t_R34.at(2);

        the2_R=atan2(t_refthe2.at(0),t_refthe2.at(1)) - atan2(sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))),-Xsw.at(2));
        //the2_R=atan2(-t_refthe2.at(0),-t_refthe2.at(1)) - atan2(sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))),Xsw.at(2));
        //the2_R = 2*atan( (t_refthe2.at(0) + ( sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))) ) ) / (Xsw.at(2) - t_refthe2.at(0)) );
        //the2_R = the2_R +(kinematics_library::PI );

        c2 = cos(the2_R); s2 = sin(the2_R);

        the1_R_var1 = (c2 * t_refthe2.at(0)) - (s2 * t_refthe2.at(1));

        c1 = Xsw.at(0) / the1_R_var1; s1 = Xsw.at(1) / the1_R_var1;

        the1_R=atan2(s1,c1);

//         std::cout<< "the1_R = "<<the1_R*kinematics_library::RTD<<"  the2_R = "<<the2_R*kinematics_library::RTD<<std::endl;
//         std::cout<<"t_refthe2= "<<t_refthe2.at(0)<<"  "<<t_refthe2[1]<<"  "<<t_refthe2.at(2)<<"  "<<(t_refthe2.at(0)+Xsw.at(2))<<"  "<<((t_refthe2.at(1)*t_refthe2.at(1))-(t_refthe2.at(0)*t_refthe2.at(0))-(Xsw.at(2)*Xsw.at(2))) <<"  "<<std::endl;
//         std::cout<< "atan2 ="<<atan2(t_refthe2.at(0),t_refthe2.at(1))*kinematics_library::RTD<<"   "<<"next= "
//                              <<atan2(sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))),-Xsw.at(2))*kinematics_library::RTD<<"  "<<
//                              ((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2)))<<"  "<<Xsw.at(2)<<std::endl;
        

        rotMatrix(the1_R, -kinematics_library::PI /2.0, R01);
        rotMatrix(the2_R, kinematics_library::PI /2.0, R12);
        rotMatrix(0.0     , -kinematics_library::PI /2.0, R23);
        multMatMat(R01, R12, t_r03_r);
        multMatMat(t_r03_r, R23, R_03_R);

        m_Xsw=sqrt( (Xsw.at(0) * Xsw.at(0)) + (Xsw.at(1) * Xsw.at(1)) + (Xsw.at(2) * Xsw.at(2)) );
        usw.at(0)=Xsw.at(0) / m_Xsw;
        usw.at(1)=Xsw.at(1) / m_Xsw;
        usw.at(2)=Xsw.at(2) / m_Xsw;

        usw_sk.at(0) =  0.0;            usw_sk.at(3) = -usw.at(2);      usw_sk.at(6) =  usw.at(1);
        usw_sk.at(1) =  usw.at(2);      usw_sk.at(4) = 0.0;             usw_sk.at(7) = -usw.at(0);
        usw_sk.at(2) = -usw.at(1);      usw_sk.at(5) = usw.at(0);       usw_sk.at(8) = 0.0;

        //Shoulder joint
        multMatMat(usw_sk, R_03_R, As);
        multMatMat(usw_sk, usw_sk, t_Bs);

        for(std::size_t i=0; i<9; i++)
        t_Bs.at(i) = -1.0 * t_Bs.at(i);

        multMatMat(t_Bs, R_03_R, Bs);
        multVecTSLvec(usw, t_Cs);                        
        multMatMat(t_Cs, R_03_R, Cs);

//         std::cout<<std::endl;
//         for(int i = 0; i < 3; i++)
//             std::cout<<R34.at(i)<<"  "<<R34.at(i+3)<<"  "<<R34.at(i+6)<<std::endl;
//         std::cout<<std::endl;

        //Wrist Joint
        transMat(R34, tsl_R34);
        transMat(As, tsl_As);
        transMat(Bs, tsl_Bs);
        transMat(Cs, tsl_Cs);

        multMatMat(tsl_R34, tsl_As, t_Aw);
        multMatMat(t_Aw, Rd, Aw);
        multMatMat(tsl_R34, tsl_Bs, t_Bw);
        multMatMat(t_Bw, Rd, Bw);
        multMatMat(tsl_R34, tsl_Cs, t_Cw);
        multMatMat(t_Cw, Rd, Cw);

//         std::cout<<"  Xsw "<<std::endl;
//         std::cout<<Xsw.at(0)<<"  "<<Xsw.at(1)<<"  "<<Xsw.at(2)<<std::endl;
// 
//         std::cout<<"  usw "<<std::endl;
//         std::cout<<usw.at(0)<<"  "<<usw.at(1)<<"  "<<usw.at(2)<<std::endl;
// 
//         std::cout<<"m_Xww = "<<m_Xsw<<std::endl;
// 
//         std::cout<<"the1_R = "<<the1_R*kinematics_library::RTD<<"   the2_R="<<the2_R*kinematics_library::RTD<<std::endl;
// 
//         std::cout<<"  R01 "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<R01.at(i)<<"  "<<R01.at(i+3)<<"  "<<R01.at(i+6)<<std::endl;
// 
//         std::cout<<"  R12 "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<R12.at(i)<<"  "<<R12.at(i+3)<<"  "<<R12.at(i+6)<<std::endl;
// 
//         std::cout<<"  R23 "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<R23.at(i)<<"  "<<R23.at(i+3)<<"  "<<R23.at(i+6)<<std::endl;
// 
//         std::cout<<"  R_03_R "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<R_03_R.at(i)<<"  "<<R_03_R.at(i+3)<<"  "<<R_03_R.at(i+6)<<std::endl;
// 
//         std::cout<<"  AS "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<As.at(i)<<"  "<<As.at(i+3)<<"  "<<As.at(i+6)<<std::endl;
// 
//         std::cout<<"  BS "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<Bs.at(i)<<"  "<<Bs.at(i+3)<<"  "<<Bs.at(i+6)<<std::endl;
// 
//         std::cout<<"  CS "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<Cs.at(i)<<"  "<<Cs.at(i+3)<<"  "<<Cs.at(i+6)<<std::endl;
// 
//         std::cout<<"  Aw "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<Aw.at(i)<<"  "<<Aw.at(i+3)<<"  "<<Aw.at(i+6)<<std::endl;
// 
//         std::cout<<"  bw "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<Bw.at(i)<<"  "<<Bw.at(i+3)<<"  "<<Bw.at(i+6)<<std::endl;
// 
//         std::cout<<"  Cw "<<std::endl;
//         for(int i = 0; i < 3; i++)
//         std::cout<<Cw.at(i)<<"  "<<Cw.at(i+3)<<"  "<<Cw.at(i+6)<<std::endl;
// 
// 
//         std::cout<<"-----------------------------------------------------------"<<std::endl;


        if(srs_config_.save_psi)
        {
            // joint 1
            save_joint_function(-As[4], -Bs[4], -Cs[4], -As[3], -Bs[3], -Cs[3],jts_limits_[0].first, jts_limits_[0].second, "joint1", "TANGENT");
            // joint 2
            save_joint_function(-As[5], -Bs[5], -Cs[5], 0, 0, 0, jts_limits_[1].first, jts_limits_[1].second, "joint2", "COSINE");
            // joint 3
            save_joint_function(As[8], +Bs[8], +Cs[8], -As[2], -Bs[2], -Cs[2], jts_limits_[2].first, jts_limits_[2].second, "joint3", "TANGENT");
            // joint 5
            save_joint_function(+Aw[7], +Bw[7], +Cw[7], +Aw[6], +Bw[6], +Cw[6], jts_limits_[4].first, jts_limits_[4].second, "joint5", "TANGENT");
            // joint 6
            save_joint_function(Aw[8], Bw[8], Cw[8], 0, 0, 0, jts_limits_[5].first, jts_limits_[5].second, "joint6", "COSINE");
            // joint 7
            save_joint_function(+Aw[5], +Bw[5], +Cw[5], -Aw[2], -Bw[2], -Cw[2], jts_limits_[6].first, jts_limits_[6].second, "joint7", "TANGENT");
        }

        //Calculating Arm Angle
        armAngle_result = cal_armangle(As,Bs,Cs,Aw,Bw,Cw,final_feasible_armangle);

        if (armAngle_result==0)
        {
            if(srs_config_.save_psi)
                save_psi_file();

            AA=final_feasible_armangle.at(0).first;

            jointangles.elements.at(0).position = atan2((-(As[4]*sin(AA))-(Bs[4]*cos(AA))-(Cs[4])),(-(As.at(3)*sin(AA))-(Bs.at(3)*cos(AA))-(Cs.at(3))));
            jointangles.elements.at(1).position = acos(-(As[5]*sin(AA))-(Bs[5]*cos(AA))-(Cs[5]));
            jointangles.elements.at(2).position = atan2(((As[8]*sin(AA))+(Bs[8]*cos(AA))+(Cs[8])),(-(As.at(2)*sin(AA))-(Bs.at(2)*cos(AA))-(Cs.at(2))));
            jointangles.elements.at(4).position = atan2(((Aw[7]*sin(AA))+(Bw[7]*cos(AA))+(Cw[7])),((Aw[6]*sin(AA))+(Bw[6]*cos(AA))+(Cw[6])));
            jointangles.elements.at(5).position = acos(((Aw[8]*sin(AA))+(Bw[8]*cos(AA))+(Cw[8])));
            jointangles.elements.at(6).position = atan2(((Aw[5]*sin(AA))+(Bw[5]*cos(AA))+(Cw[5])),(-(Aw.at(2)*sin(AA))-(Bw.at(2)*cos(AA))-(Cw.at(2))));
            
            for(int i = 0; i < 7; i++)
                std::cout<<jointangles.elements.at(i).position*kinematics_library::RTD<<"  ";
            std::cout<<std::endl;
        }
        else
            succeeded = armAngle_result;
    }

    feasible_psi.clear();
    infeasible_psi.clear();
    final_feasible_armangle.clear();

    return succeeded;

}

int SRSKinematicSolver::cal_armangle(   const std::vector<double> &As, const std::vector<double> &Bs, const std::vector<double> &Cs,
                                        const std::vector<double> &Aw, const std::vector<double> &Bw, const std::vector<double> &Cw,
                                        std::vector< std::pair<double,double>  > &final_feasible_armangle)
{
    int succeeded = 0;
    double at1=0.0, bt1=0.0, ct1=0.0, at3=0.0, bt3=0.0, ct3=0.0, at5=0.0, bt5=0.0, ct5=0.0, at7=0.0, bt7=0.0, ct7=0.0;
    double cond1=0.0, cond3=0.0, cond5=0.0, cond7=0.0;

    at1=((-Bs[3])*(-Cs[4]))-((-Bs[4])*(-Cs[3]));
    bt1=((-As[4])*(-Cs[3]))-((-As[3])*(-Cs[4]));
    ct1=((-As[4])*(-Bs[3]))-((-As[3])*(-Bs[4]));

    at3=((-Bs[2])*(Cs[8]))-((Bs[8])*(-Cs[2]));
    bt3=((As[8])*(-Cs[2]))-((-As[2])*(-Cs[8]));
    ct3=((As[8])*(-Bs[2]))-((-As[2])*(-Bs[8]));

    at5=((Bw[6])*(Cw[7]))-((Bw[7])*(Cw[6]));
    bt5=((Aw[7])*(Cw[6]))-((Aw[6])*(Cw[7]));
    ct5=((Aw[7])*(Bw[6]))-((Aw[6])*(Bw[7]));

    at7=((-Bw[2])*(Cw[5]))-((Bw[5])*(-Cw[2]));
    bt7=((Aw[5])*(-Cw[2]))-((-Aw[2])*(Cw[5]));
    ct7=((Aw[5])*(-Bw[2]))-((-Aw[2])*(Bw[5]));

    //condition

    cond1=(at1*at1)+(bt1*bt1)-(ct1*ct1);
    cond3=(at3*at3)+(bt3*bt3)-(ct3*ct3);
    cond5=(at5*at5)+(bt5*bt5)-(ct5*ct5);
    cond7=(at7*at7)+(bt7*bt7)-(ct7*ct7);

    //Joint 1
    succeeded = tangenttype_armangle(-As[4], -Bs[4], -Cs[4], -As[3], -Bs[3], -Cs[3], at1, bt1, ct1, cond1, 1, jts_limits_[0].first, jts_limits_[0].second, "joint1");
    // if (succeeded != 0) return succeeded;

    //Joint 2
    succeeded = feasible_armangle_cosinetype_cyclicfunction(-As[5], -Bs[5], -Cs[5], jts_limits_[1].first, jts_limits_[1].second, 2, "joint2");
    //if (succeeded != 0) return succeeded;

    //Joint 3
    succeeded = tangenttype_armangle(As[8], +Bs[8], +Cs[8], -As[2], -Bs[2], -Cs[2], at3, bt3, ct3, cond3, 3, jts_limits_[2].first, jts_limits_[2].second, "joint3");
    //if (succeeded != 0)  return succeeded;

    //Joint 5
    succeeded = tangenttype_armangle(+Aw[7], +Bw[7], +Cw[7], +Aw[6], +Bw[6], +Cw[6], at5, bt5, ct5, cond5, 5, jts_limits_[4].first, jts_limits_[4].second, "joint5");
    //if (succeeded != 0)  return succeeded;

    //Joint 6    
    succeeded = feasible_armangle_cosinetype_cyclicfunction(Aw[8], Bw[8], Cw[8], jts_limits_[5].first, jts_limits_[5].second, 6, "joint6");
    // if (succeeded != 0) return succeeded;

    //Joint 7
    succeeded = tangenttype_armangle(+Aw[5], +Bw[5], +Cw[5], -Aw[2], -Bw[2], -Cw[2], at7, bt7, ct7, cond7, 7, jts_limits_[6].first, jts_limits_[6].second, "joint7");
    // if (succeeded != 0) return succeeded;

    // debugging
    std::cout<<"Feasible Psi "<<std::endl;
    for(size_t i = 0; i< feasible_psi.size(); i++)
    {
        for(size_t j = 0; j< feasible_psi.at(i).psi.size(); j++)
            std::cout<<"Joint "<<feasible_psi.at(i).joint_number<<"  "<<feasible_psi.at(i).joint_name<<"  "<< "Psi = "<<feasible_psi.at(i).psi.at(j).first*kinematics_library::RTD<<"  "<<feasible_psi.at(i).psi.at(j).second*kinematics_library::RTD<<std::endl;
    }

    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Infeasible Psi "<<std::endl;
    for(size_t i = 0; i< infeasible_psi.size(); i++)
    {                        
        for(size_t j = 0; j< infeasible_psi.at(i).psi.size(); j++)
            std::cout<<"Joint "<<infeasible_psi.at(i).joint_number<<"  "<<infeasible_psi.at(i).joint_name<<"  "<< "Psi = "<<infeasible_psi.at(i).psi.at(j).first*kinematics_library::RTD<<"  "<<infeasible_psi.at(i).psi.at(j).second*kinematics_library::RTD<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<std::endl;
    

    //std::pair<double, double> single_feasible_result;
    std::vector< ArmAngle > single_feasible_result;
    int intesection_result = union_joints_with_only_one_feasible_armangle(feasible_psi, single_feasible_result);


    std::cout<<"intersected Feasible Psi "<<intesection_result<<std::endl;

    for(size_t i = 0; i< single_feasible_result.size(); i++)
    {
        for(size_t j = 0; j< single_feasible_result.at(i).psi.size(); j++)
            std::cout<<"Joint "<<single_feasible_result.at(i).joint_number<<"  "<<single_feasible_result.at(i).joint_name<<"    "<< single_feasible_result.at(i).psi.at(j).first*kinematics_library::RTD<<"  "<<single_feasible_result.at(i).psi.at(j).second*kinematics_library::RTD<<std::endl;
    }


    complement_of_infeasible_psi(infeasible_psi, single_feasible_result);

    succeeded = union_of_all_feasible_armangle(single_feasible_result, final_feasible_armangle);
    if( succeeded != 0)
        return succeeded;

    
    LOG_DEBUG("[SRSKinematicSolver]: compliment infeasbile psi");

    for(std::size_t i = 0; i< single_feasible_result.size(); i++)
    {
        for(std::size_t j = 0; j< single_feasible_result.at(i).psi.size(); j++)
            LOG_DEBUG("[SRSKinematicSolver]: Joint %i %s %f %f",single_feasible_result.at(i).joint_number, single_feasible_result.at(i).joint_name.c_str(), 
                single_feasible_result.at(i).psi.at(j).first*kinematics_library::RTD,single_feasible_result.at(i).psi.at(j).second*kinematics_library::RTD);
    }

    LOG_DEBUG("[SRSKinematicSolver]: Final feasbile psi %i", final_feasible_armangle.size());

    for(std::size_t i = 0; i< final_feasible_armangle.size(); i++)
        LOG_DEBUG("[SRSKinematicSolver]: %f %f",final_feasible_armangle.at(i).first*kinematics_library::RTD, final_feasible_armangle.at(i).second*kinematics_library::RTD);

    return succeeded;
}

int SRSKinematicSolver::tangenttype_armangle(   const double &an, const double bn, const double cn,
                                                const double &ad, const double &bd, const double &cd,
                                                const double &at, const double &bt, const double &ct,
                                                const double &condition, const int &jointnumber,
                                                const double &min_jointlimit, const double &max_jointlimit,
                                                const std::string& jointname)
{
    int succeeded = 0;

    LOG_DEBUG("[SRSKinematicSolver]: Joint number %i with condition %f", jointnumber, condition);
    

    if ((condition < kinematics_library::PAC) && (condition > kinematics_library::NAC))
    {    
        //std::cout<<"Cond 1"<<std::endl;
        succeeded = feasible_armangle_tangenttype_stationarycase(an, bn, cn, ad, bd, cd, at, bt, ct, jointnumber, jointname);

        if (succeeded != 0)
            return succeeded;
    }
    else if (condition > 0)
    {    
//         std::cout<<"Cond 2"<<std::endl;
        succeeded = feasible_armangle_tangenttype_cyclicfunction(an, bn, cn, ad, bd, cd, at, bt, ct, min_jointlimit, max_jointlimit, jointnumber, jointname);

        if (succeeded != 0)
            return succeeded;

    }
    else if (condition < 0)
    {
//         std::cout<<"Cond 3"<<std::endl;
        succeeded = feasible_armangle_monotonicfunction(an, bn, cn, ad, bd, cd, min_jointlimit, max_jointlimit, jointnumber, jointname);

        if (succeeded != 0)
            return succeeded;
    }
    return succeeded;

}

int SRSKinematicSolver::feasible_armangle_tangenttype_stationarycase(const double &an, const double &bn, const double &cn,
                                                                    const double &ad, const double &bd, const double &cd,
                                                                    const double &at, const double &bt, const double &ct,
                                                                    const int &jointnumber, const std::string& jointname)
{
    double psi_stationary = 0.0;
//     double left_jtlimit = 0.0, right_jtlimit = 0.0;
    ArmAngle calculated_psi;

    psi_stationary = 2 * atan( at / (bt - ct) );

    calculated_psi.joint_name = jointname;
    calculated_psi.joint_number = jointnumber;
    calculated_psi.psi.resize(1);
    calculated_psi.psi.at(0) = std::make_pair(psi_stationary,psi_stationary);

    infeasible_psi.push_back(calculated_psi);

    //infeasible_psi.push_back(std::make_pair(psi_stationary,psi_stationary));
    //todo: find the use of this limit

//     /*left_jtlimit    = atan2( (( (at * bn) - (bt * an)) / -ct), ( ((at * bd) - (bt * ad)) / -ct ) );
//     right_jtlimit   = atan2( (( (at * bn) - (bt * an)) /  ct), ( ((at * bd) - (bt * ad)) /  ct ) );*/

    LOG_DEBUG("[SRSKinematicSolver]: SINGULAR CASE with psi_stationary %f", psi_stationary*kinematics_library::RTD);

    return 0;
}

int SRSKinematicSolver::feasible_armangle_monotonicfunction(const double &an, const double &bn, const double &cn,
                                                            const double &ad, const double &bd, const double &cd,
                                                            const double &min_jtag, const double &max_jtag,
                                                            const int &jointnumber, const std::string& jointname)
{
    std::pair<double, double> transcendental_solution(0.0, 0.0);
    double a = 0.0, b = 0.0, c = 0.0;
    ArmAngle calculated_psi;

    int succeeded = 0; // 0 = success, -1 = no feasible arm angle

    LOG_DEBUG("[SRSKinematicSolver]: MONOTONIC \n");

    // +90 and -90 can be made in to one if loop
    if (fabs(min_jtag - kinematics_library::PI /2.0) < kinematics_library::EPSILON) //+90
    {
        a = bd;
        b = ad;
        c = -cd;
    }
    else if (fabs(min_jtag + kinematics_library::PI /2.0) < kinematics_library::EPSILON) //-90
    {
        a = -bd;
        b = -ad;
        c = cd;
    }
    else if ((fabs(min_jtag - kinematics_library::PI ) < kinematics_library::EPSILON) || (fabs(min_jtag + kinematics_library::PI ) < kinematics_library::EPSILON) ) //180|| -180
    {
        a =  bn;
        b =  an;
        c = -cn;
    }
    else
    {
        a = (tan(min_jtag) * bd) - bn;
        b = (tan(min_jtag) * ad) - an;
        c = cn - (tan(min_jtag) * cd);
        /*a = (min_jtag * bd) - bn;
        b = (min_jtag * ad) - an;
        c = cn - (min_jtag * cd);*/
    }

    succeeded = solve_transcendental_equation(a, b, c, transcendental_solution);
    if( succeeded != 0)
        return succeeded;

    calculated_psi.joint_name = jointname;
    calculated_psi.joint_number = jointnumber;
    calculated_psi.psi.resize(1);

    succeeded = psi_picker(transcendental_solution, "MIN", calculated_psi.psi.at(0).first);
    //succeeded = solve_transcendental_equation(a, b, c, calculated_psi.psi.at(0));
    if( succeeded != 0)
        return succeeded;

    //feasible_psi.push_back(calculated_psi);
    LOG_DEBUG("[SRSKinematicSolver]: Calculated PSI %f", calculated_psi.psi.at(0).first*kinematics_library::RTD);
    
    if ( fabs(max_jtag - kinematics_library::PI /2.0) < kinematics_library::EPSILON) //+90
    {
        a = bd; b = ad; c = -cd;
    }
    else if (fabs(max_jtag + kinematics_library::PI /2.0) < kinematics_library::EPSILON) //-90
    {
        a = -bd; b = -ad; c = cd;
    }
    else if ((fabs(max_jtag - kinematics_library::PI ) < kinematics_library::EPSILON) || (fabs(max_jtag + kinematics_library::PI ) < kinematics_library::EPSILON) ) //180|| -180
    {
        a =  bn; b =  an; c = -cn;
    }
    else
    {
        a = (tan(max_jtag) * bd) - bn;
        b = (tan(max_jtag) * ad) - an;
        c = cn - (tan(max_jtag) * cd);
        /*a = (max_jtag * bd) - bn;
        b = (max_jtag * ad) - an;
        c = cn - (max_jtag * cd);*/
    }

    succeeded = solve_transcendental_equation(a, b, c, transcendental_solution);
    if( succeeded != 0)
        return succeeded;

    succeeded = psi_picker(transcendental_solution, "MAX", calculated_psi.psi.at(0).second);
    //succeeded = solve_transcendental_equation(a, b, c, calculated_psi.psi.at(0));

    if( succeeded != 0 )
        return succeeded;

    feasible_psi.push_back(calculated_psi);

    LOG_DEBUG("[SRSKinematicSolver]: Calculated PSI %f", calculated_psi.psi.at(0).second*kinematics_library::RTD);

    return succeeded;

}

int SRSKinematicSolver::feasible_armangle_cosinetype_cyclicfunction(const double &at, const double &bt, const double &ct,
                                                                    const double &min_jtag, const double &max_jtag,
                                                                    const int &jointnumber, const std::string& jointname)
{
    double global_min = 0.0, global_max = 0.0;
    double t_sqrt = 0.0;    // temporary variable
    double psi_min = 0.0, psi_max = 0.0;
    std::pair<double, double> psi(0.0, 0.0);
    double a = 0.0, b = 0.0, c = 0.0;
    ArmAngle calculated_psi;
    int succeeded = 0; // 0 = success, -1 = no feasible arm angle
    bool swap_flag = false;

    LOG_DEBUG("[SRSKinematicSolver]: COSINE CYCLIC \n");

    t_sqrt = sqrt((at * at) + (bt * bt));

    if(fabs(at) <= kinematics_library::EPSILON)
    {
        psi_min = kinematics_library::PI ;
        psi_max = 0.0;
        std::cout<<"   a is zero "<<std::endl;
    }
    else
    {
        psi_min = (2 * atan((-bt - t_sqrt) / at));
        psi_max = (2 * atan((-bt + t_sqrt) / at));
    }

    global_min = acos((at * sin(psi_min)) + (bt * cos(psi_min)) + ct);
    global_max = acos((at * sin(psi_max)) + (bt * cos(psi_max)) + ct);

    LOG_DEBUG("[SRSKinematicSolver]: global_min = %f global_max = %f", global_min*kinematics_library::RTD, global_max*kinematics_library::RTD);

    if(global_min > global_max)
    {
        std::swap(global_min, global_max);
        swap_flag = true;

        LOG_DEBUG("[SRSKinematicSolver]: \n Swapping global_min = %f global_max = %f \n", global_min*kinematics_library::RTD, global_max*kinematics_library::RTD);
    }

//     std::cout<< t_sqrt<<"  "<<at<<"  "<<bt<<"  "<<ct<<"  "<<kinematics_library::EPSILON<<std::endl;
//     std::cout<<( (at * at) + (bt * bt) - ((ct-1.0) * (ct-1.0)) ) <<std::endl;
//     std::cout<<( (at * at) + (bt * bt) - ((ct+1.0) * (ct+1.0)) )<<std::endl;
//     std::cout<<"global_min = "<<global_min*kinematics_library::RTD<<"  global_max = "<<global_max*kinematics_library::RTD<<std::endl;
//     std::cout<<"psi_min = "<<psi_min*kinematics_library::RTD<<"  psi_max = "<<psi_max*kinematics_library::RTD<<std::endl;

    calculated_psi.joint_name = jointname;
    calculated_psi.joint_number = jointnumber;

    //if ((fabs( (at * at) + (bt * bt) - ((ct-1) * (ct-1)) )) < kinematics_library::EPSILON)
    if ( (((at * at) + (bt * bt) - ((ct-1) * (ct-1)) ) <kinematics_library::PAC ) && (((at * at) + (bt * bt) - ((ct-1) * (ct-1)) ) >kinematics_library::NAC))
    {
        psi.first  = 2 * atan(at / (bt - (ct - 1)));
        psi.second = psi.first;

        calculated_psi.psi.resize(1);
        calculated_psi.psi.at(0) = std::make_pair(psi.first, psi.second);

        if(swap_flag)
            feasible_psi.push_back(calculated_psi);
        else
            infeasible_psi.push_back(calculated_psi);

    }
    else if (fabs(( (at * at) + (bt * bt) - ((ct+1) * (ct+1)) )) < kinematics_library::EPSILON)
    {
        psi.first  = 2 * atan(at / (bt - (ct + 1)));
        psi.second = psi.first;

        calculated_psi.psi.resize(1);
        calculated_psi.psi.at(0) = std::make_pair(psi.first, psi.second);

        if(swap_flag)
            feasible_psi.push_back(calculated_psi);
        else
            infeasible_psi.push_back(calculated_psi);

    }
    else
    {
        //boundry condition
        a = bt;
        b = at;
        //cond-1
        if( (global_min > max_jtag) || (global_max < min_jtag) )
        {
//             std::cout<< "cond-1"<<std::endl;
            succeeded = -(jointnumber);

            LOG_DEBUG("[SRSKinematicSolver]: Failed in boundary condition in cosine function");
            LOG_DEBUG("[SRSKinematicSolver]: psi_min = %f psi_max = %f",psi_min*kinematics_library::RTD, psi_max*kinematics_library::RTD );
            LOG_DEBUG("[SRSKinematicSolver]: global_min = %f global_max = %f", global_min*kinematics_library::RTD, global_max*kinematics_library::RTD);
            LOG_DEBUG("[SRSKinematicSolver]: psi.first = %f psi.second = %f", psi.first*kinematics_library::RTD, psi.second*kinematics_library::RTD);

            return succeeded;
        }
        //cond-2
        else if( (global_min < min_jtag) && (( min_jtag <= global_max) && (global_max <= max_jtag)) )
        {
//             std::cout<< "cond-2"<<std::endl;
            // solve equation 24
            c = cos(min_jtag) - ct;

            //succeeded = solve_transcendental_equation(a, b, c, psi);
            succeeded = solve_transcendental_equation_newmethod(a, b, c, psi);
            if( succeeded != 0)
                return succeeded;
            succeeded = check_psi_range_bw_negPI_posPI(psi);
            if( succeeded != 0)
                return succeeded;

            calculated_psi.psi.resize(1);
            calculated_psi.psi.at(0) = std::make_pair(psi.first, psi.second);

            if(swap_flag)
                feasible_psi.push_back(calculated_psi);
            else
                infeasible_psi.push_back(calculated_psi);
        }
        //cond-3
        else if( ((min_jtag <= global_min) && (global_min <= max_jtag)) && (global_max >= max_jtag) )
        {
            // solve equation 24
            c = cos(max_jtag) - ct;

//             std::cout<< "cond-3"<<std::endl;
//             std::cout<<"a = "<< a <<"   "<<"b = "<<b<<"  "<<"c = "<<c<<std::endl;            

            succeeded = solve_transcendental_equation(a, b, c, psi);
            //succeeded = solve_transcendental_equation_newmethod(a, b, c, psi);
            if( succeeded != 0)
                return succeeded;
            succeeded = check_psi_range_bw_negPI_posPI(psi);
            if( succeeded != 0)
                return succeeded;

            calculated_psi.psi.resize(1);
            calculated_psi.psi.at(0) = std::make_pair(psi.first, psi.second);

            if(swap_flag)
                feasible_psi.push_back(calculated_psi);
            else
                infeasible_psi.push_back(calculated_psi);

        }
        //cond-4
        else if( (global_min < min_jtag) && (global_max > max_jtag ) )
        {
//             std::cout<< "cond-4"<<std::endl;
            // solve equation 24
            c = cos(min_jtag) - ct;

            succeeded = solve_transcendental_equation(a, b, c, psi);
            if( succeeded != 0)
                return succeeded;
            succeeded = check_psi_range_bw_negPI_posPI(psi);
            if( succeeded != 0)
                return succeeded;

            calculated_psi.psi.resize(2);
            calculated_psi.psi.at(0) = std::make_pair(psi.first, psi.second);

            if(swap_flag)
                feasible_psi.push_back(calculated_psi);
            else
                infeasible_psi.push_back(calculated_psi);

            // solve equation 24
            c = cos(max_jtag) - ct;

            succeeded = solve_transcendental_equation(a, b, c, psi);
            if( succeeded != 0)
                return succeeded;

            succeeded = check_psi_range_bw_negPI_posPI(psi);
            if( succeeded != 0)
                return succeeded;

            calculated_psi.psi.at(1) = std::make_pair(psi.first, psi.second);

            if(swap_flag)
                feasible_psi.push_back(calculated_psi);
            else
                infeasible_psi.push_back(calculated_psi);

        }
        //cond-5
        else if( ((min_jtag <= global_min) && (global_min <= max_jtag )) &&
                ((min_jtag <= global_max) && (global_max <= max_jtag )) )
        {
//             std::cout<< "cond-5"<<std::endl;

            calculated_psi.psi.resize(1);
            calculated_psi.psi.at(0) = std::make_pair(-kinematics_library::PI , kinematics_library::PI );

            feasible_psi.push_back(calculated_psi);
        }
    }

//     std::cout<<"at = "<< at <<"   "<<"bt = "<<bt<<"  "<<"ct = "<<ct<<std::endl;
//     std::cout<<"cond 1 = "<<( (at * at) + (bt * bt) - ((ct-1) * (ct-1)) )<<"  "<<"cond 2 = "<<( (at * at) + (bt * bt) - ((ct+1) * (ct+1)) )<<std::endl;
//     std::cout<<"t_sqrt = "<< t_sqrt <<"   "<<"squared = "<<((at * at) + (bt * bt))<<std::endl;
//     std::cout<<"psi_min = "<<psi_min*kinematics_library::RTD<<"  psi_max = "<<psi_max*kinematics_library::RTD<<std::endl;

    return succeeded;
}

int SRSKinematicSolver::feasible_armangle_tangenttype_cyclicfunction(const double &an, const double &bn, const double &cn,
                                                                    const double &ad, const double &bd, const double &cd,
                                                                    const double &at, const double &bt, const double &ct,
                                                                    const double &min_jtag, const double &max_jtag,
                                                                    const int &jointnumber, const std::string& jointname)
{
    double global_min = 0.0, global_max = 0.0;
    double t_sqrt = 0.0;    // temporary variable
    double psi_min = 0.0, psi_max = 0.0;                
    ArmAngle calculated_psi;
    int succeeded = 0; // 0 = success, -1 = no feasible arm angle

    LOG_DEBUG("[SRSKinematicSolver]:TANGENT CYCLIC \n");

    t_sqrt = sqrt( (at * at) + (bt * bt) - (ct * ct) );
    psi_min = 2.0*(atan( (at-t_sqrt) / (bt-ct)) );
    psi_max = 2.0*(atan( (at+t_sqrt) / (bt-ct)) );

    LOG_DEBUG("[SRSKinematicSolver]: psi_min = %f psi_max = %f",psi_min*kinematics_library::RTD, psi_max*kinematics_library::RTD );

    global_min = atan2( ( (an * sin(psi_min))+(bn * cos(psi_min))+cn ), ( (ad * sin(psi_min))+(bd * cos(psi_min))+cd ) );
    global_max = atan2( ( (an * sin(psi_max))+(bn * cos(psi_max))+cn ), ( (ad * sin(psi_max))+(bd * cos(psi_max))+cd ) );

    /*if(global_min > global_max)
    {
            swap(global_min, global_max);

            if(DEBUG)
                    std::cout<< "*************   swaping **********"<<"global_min "<<global_min*kinematics_library::RTD<<" "<<"global_max "<<global_max*kinematics_library::RTD<<std::endl;

    }*/

    LOG_DEBUG("[SRSKinematicSolver]: global_min = %f global_max = %f", global_min*kinematics_library::RTD, global_max*kinematics_library::RTD);

    calculated_psi.joint_name = jointname;
    calculated_psi.joint_number = jointnumber;

    //boundry condition
    if( (global_min > max_jtag) || (global_max < min_jtag) )                                                //cond-1
    {
            succeeded = -(jointnumber);                        
            return succeeded;
    }
    else if( (global_min < min_jtag) && (( min_jtag <= global_max) && (global_max <= max_jtag)) )           //cond-2
    {
        
//         std::cout<< "cond-2"<<std::endl;
        calculated_psi.psi.resize(1);
        //calculated_psi.psi.at(0) = std::make_pair(0.0, 0.0);

        succeeded = calculate_region_armAngle_tangentcylic(an, bn, cn, ad, bd, cd, min_jtag, calculated_psi.psi.at(0) );
        if( succeeded != 0)
                return succeeded;

        infeasible_psi.push_back(calculated_psi);
    }
    else if( ((min_jtag <= global_min) && (global_min <= max_jtag)) && (global_max >= max_jtag) )           //cond-3
    {
//         std::cout<< "cond-3"<<std::endl;
        calculated_psi.psi.resize(1);

        succeeded = calculate_region_armAngle_tangentcylic(an, bn, cn, ad, bd, cd, max_jtag, calculated_psi.psi.at(0) );
        if( succeeded != 0)
                return succeeded;

        /*if(!check_for_psi_range(calculated_psi.psi.at(0)))
        {std::cout<< "%%%%%%%%%%%%%%%%%%%%%%%%%"<<std::endl;
                succeeded = calculate_region_armAngle_tangentcylic(an, bn, cn, ad, bd, cd, min_jtag, calculated_psi.psi.at(0) );
                feasible_psi.push_back(calculated_psi);

        }
        else
                infeasible_psi.push_back(calculated_psi);*/

        infeasible_psi.push_back(calculated_psi);
    }
    else if( (global_min < min_jtag) && (global_max > max_jtag ) )                                          //cond-4
    {
//         std::cout<< "cond-4"<<std::endl;
        calculated_psi.psi.resize(1);
        //calculated_psi.psi.at(0) = std::make_pair(0.0, 0.0);

        succeeded = calculate_region_armAngle_tangentcylic(an, bn, cn, ad, bd, cd, min_jtag, calculated_psi.psi.at(0) );
        if( succeeded != 0)
                return succeeded;

        infeasible_psi.push_back(calculated_psi);

        succeeded = calculate_region_armAngle_tangentcylic(an, bn, cn, ad, bd, cd, max_jtag, calculated_psi.psi.at(0) );
        if( succeeded != 0)
                return succeeded;

        infeasible_psi.push_back(calculated_psi);

    }
    else if( ((min_jtag <= global_min) && (global_min <= max_jtag )) && 
             ((min_jtag <= global_max) && (global_max <= max_jtag )) )                          //cond-5
    {
        calculated_psi.psi.resize(1);
        calculated_psi.psi.at(0) = std::make_pair(-kinematics_library::PI , kinematics_library::PI );

        feasible_psi.push_back(calculated_psi);
    }

    LOG_DEBUG("[SRSKinematicSolver]: psi_min = %f psi_max = %f",psi_min*kinematics_library::RTD, psi_max*kinematics_library::RTD );
    LOG_DEBUG("[SRSKinematicSolver]: global_min = %f global_max = %f", global_min*kinematics_library::RTD, global_max*kinematics_library::RTD);

    return succeeded;
}

int SRSKinematicSolver::calculate_region_armAngle_tangentcylic(const double &an, const double &bn, const double &cn,
                                                            const double &ad, const double &bd, const double &cd,
                                                            const double &joint_limit, std::pair< double,double > &psi_pair )
{
    double a = 0.0, b = 0.0, c = 0.0;
    std::pair<double, double> psi(0.0, 0.0);
    int succeeded = 0;

    // solve equation 23
    // since tan(90) is infinity, the equ 23 is modified
    if (fabs(joint_limit - kinematics_library::PI /2.0) < kinematics_library::EPSILON) //+90
    {
        a = bd;
        b = ad;
        c = -cd;
    }
    else if (fabs(joint_limit + kinematics_library::PI /2.0) < kinematics_library::EPSILON) //-90
    {
        a = -bd;
        b = -ad;
        c = cd;
    }
    else
    {
        a = (tan(joint_limit) * bd) - bn;
        b = (tan(joint_limit) * ad) - an;
        c = cn - (tan(joint_limit) * cd);

        /*a = (joint_limit * bd) - bn;
        b = (joint_limit * ad) - an;
        c = cn - (joint_limit * cd);*/
    }

    succeeded = solve_transcendental_equation(a, b, c, psi);
    //std::cout<< "-----------------------  "<<"  "<<"a ="<<ad<<"  b="<<bd<<"  c="<<cd<<std::endl;
    if( succeeded != 0)
    return succeeded;

    succeeded = check_psi_range_bw_negPI_posPI(psi);
    if( succeeded != 0)
        return succeeded;

    psi_pair = std::make_pair(psi.first, psi.second);

    LOG_DEBUG("[SRSKinematicSolver]: psi.first = %f psi.second = %f",psi.first*kinematics_library::RTD, psi.second*kinematics_library::RTD );

    return succeeded;
}

int SRSKinematicSolver::solve_transcendental_equation(const double a, const double b, const double c, std::pair<double, double>& transcendental_solutions)
{

    double psi_1 = 0.0, psi_2 = 0.0;
    int succeeded = 0;
    double squared_value = ((a * a) + (b * b) - (c * c)) ;

    // todo
    /*if(squared_value < 0)
    {
            if(DEBUG)
            {
                    std::cout<< "transc_equ--  "<<"squared_value ="<<squared_value<<"   a ="<<a<<"  b="<<b<<"  c="<<c<<std::endl;
                    succeeded = kinematics_library::ERR_TRANS_EQU;
            }
    }*/

    if ( fabs(b) < kinematics_library::EPSILON) //~0
    {
        LOG_DEBUG("[SRSKinematicSolver]: transcendental_solutions b is zero ");
        
        double temp = c/a;
        //transcendental_solutions.first = atan2(temp,-sqrt(1-(temp*temp)));
        //transcendental_solutions.second = atan2(temp,sqrt(1-(temp*temp)));

        transcendental_solutions.first = -acos(temp);
        transcendental_solutions.second = acos(temp);

//         std::cout<< "transc_equ b zero "<<sqrt(fabs(1-(temp*temp)))<<"  "<<temp<<"  "<<atan2(temp,sqrt(1-(temp*temp)))*kinematics_library::RTD<<std::endl;
    }
    else if ( fabs(c) < kinematics_library::EPSILON) //~0
    {
        transcendental_solutions.first = atan2(a,-b);
        transcendental_solutions.second = atan2(-a,b);

        LOG_DEBUG("[SRSKinematicSolver]: transcendental_solutions ' c<0 ' condition ");

        succeeded = SRSKinematic::ERR_TRANS_EQU_COND;
    }
    else
    {
        psi_1 = atan2(b,a);
        psi_2 = atan2(sqrt(squared_value), c);

        transcendental_solutions.first  = psi_1 - psi_2;
        transcendental_solutions.second = psi_1 + psi_2;
    }
    
    LOG_DEBUG("[SRSKinematicSolver]: transc_equ: psi_1 = %f psi_2 = %f sqrt = %f a = %f b = %f c = %f",psi_1*kinematics_library::RTD, psi_2*kinematics_library::RTD, 
                                                                                                                  sqrt(((a * a) + (b * b) - (c * c)) ), a, b, c);
    LOG_DEBUG("[SRSKinematicSolver]: transc_equ solu %f %f", transcendental_solutions.first*kinematics_library::RTD, transcendental_solutions.second*kinematics_library::RTD);
    
    return succeeded;
}

int SRSKinematicSolver::solve_transcendental_equation_newmethod(const double a, const double b, const double c, std::pair<double, double>& transcendental_solutions)
{

    double R = 0.0;
    double sin_alpha = 0.0, cos_alpha = 0.0, alpha = 0.0;
    double temp_cos = 0.0;

    int succeeded = 0;

    R = sqrt((a * a) + (b * b));
    sin_alpha = b / R;
    cos_alpha = a / R;
    alpha = atan2(sin_alpha, cos_alpha);
    temp_cos = c / R;

    if (temp_cos >= 1)
    {
        transcendental_solutions.first  = alpha;
        transcendental_solutions.second = alpha;
    }
    else if (temp_cos <= -1)
    {
        transcendental_solutions.first  = kinematics_library::PI + alpha;
        transcendental_solutions.second = kinematics_library::PI + alpha;
    }
    else
    {
        transcendental_solutions.first  = acos(temp_cos) + alpha;
        transcendental_solutions.second = acos(temp_cos) + alpha;
    }
    
    LOG_DEBUG("[SRSKinematicSolver]: new transc_equ: R = %f sin_alpha = %f cos_alpha = %f alpha = %f a = %f b = %f c = %f",R, sin_alpha, cos_alpha, 
                                                                                                                          alpha*kinematics_library::RTD, a, b, c);
    LOG_DEBUG("[SRSKinematicSolver]: new transc_equ solu %f %f", transcendental_solutions.first*kinematics_library::RTD, transcendental_solutions.second*kinematics_library::RTD);
    
    return succeeded;
}

int SRSKinematicSolver::psi_picker(std::pair<double, double> act_psi, const std::string& limit, double &result)
{
    double temp_psi_first  = 0.0;
    double temp_psi_second = 0.0;
    int succeeded = 0;

    if (limit == "MIN")
    {
        if( ((act_psi.first > -kinematics_library::PI ) && (act_psi.first < kinematics_library::EPSILON) ) && ((act_psi.second < -kinematics_library::PI ) || (act_psi.second > kinematics_library::EPSILON)) )
        {
            result = act_psi.first;
            return succeeded;
        }
        else if ( ((act_psi.first < -kinematics_library::PI ) || (act_psi.first > kinematics_library::EPSILON))&& ((act_psi.second < kinematics_library::EPSILON) &&(act_psi.second > -kinematics_library::PI )) )
        {
            result = act_psi.second;
            return succeeded;
        }
        else if ((act_psi.first > kinematics_library::EPSILON) && (act_psi.second > kinematics_library::EPSILON))
        {
            temp_psi_first = act_psi.first - (2*kinematics_library::PI );
            temp_psi_second = act_psi.second - (2*kinematics_library::PI );
            if( ((temp_psi_first > -kinematics_library::PI ) && (temp_psi_first < kinematics_library::EPSILON) ) && ((temp_psi_second < -kinematics_library::PI ) || (temp_psi_second > kinematics_library::EPSILON)) )
            {
                result = temp_psi_first;
                return succeeded;
            }
            else if ( ((temp_psi_first < -kinematics_library::PI ) || (temp_psi_first > kinematics_library::EPSILON))&& ((temp_psi_second < kinematics_library::EPSILON) &&(temp_psi_second > -kinematics_library::PI )) )
            {
                result = temp_psi_second;
                return succeeded;
            }
            else
            {
                return SRSKinematic::ERR_TRANS_EQU_PICK_1;
            }
        }
        else
        {
        return SRSKinematic::ERR_TRANS_EQU_PICK_2;
        }
    }
    else if(limit == "MAX")
    {
        if( ((act_psi.first < kinematics_library::PI ) && (act_psi.first > kinematics_library::EPSILON) ) && ((act_psi.second > kinematics_library::PI ) || (act_psi.second < kinematics_library::EPSILON)) )
        {
            result = act_psi.first;
            return succeeded;
        }
        else if ( ((act_psi.first > kinematics_library::PI ) || (act_psi.first < kinematics_library::EPSILON)) && ((act_psi.second > kinematics_library::EPSILON) && (act_psi.second < kinematics_library::PI )) )
        {
            result = act_psi.second;
            return succeeded;
        }
        else if ((act_psi.first < kinematics_library::EPSILON) && (act_psi.second < kinematics_library::EPSILON))
        {
            temp_psi_first = act_psi.first + (2*kinematics_library::PI );
            temp_psi_second = act_psi.second + (2*kinematics_library::PI );
            if( ((temp_psi_first < kinematics_library::PI ) && (temp_psi_first > kinematics_library::EPSILON) ) && ((temp_psi_second > kinematics_library::PI ) || (temp_psi_second < kinematics_library::EPSILON)) )
            {
                result = temp_psi_first;
                return succeeded;
            }
            else if ( ((temp_psi_first > kinematics_library::PI ) || (temp_psi_first < kinematics_library::EPSILON)) && ((temp_psi_second > kinematics_library::EPSILON) && (temp_psi_second < kinematics_library::PI )) )
            {
                result = temp_psi_second;
                return succeeded;
            }
            else
            {
            return SRSKinematic::ERR_TRANS_EQU_PICK_3;
            }
        }
        else
        {
            return SRSKinematic::ERR_TRANS_EQU_PICK_4;
        }
    }
    else
    {
        return SRSKinematic::ERR_TRANS_EQU_PICK_5;
    }
}

int SRSKinematicSolver::check_psi_range_bw_negPI_posPI(std::pair<double, double> &act_psi)
{
    //if( ((act_psi.first > -PI) && (act_psi.first < kinematics_library::EPSILON) ) &&
    //    ((act_psi.second < PI) && (act_psi.second > kinematics_library::EPSILON)) )
    if( ((act_psi.first > -kinematics_library::PI ) && (act_psi.first < kinematics_library::PI ) ) &&
        ((act_psi.second > -kinematics_library::PI ) && (act_psi.second < kinematics_library::PI )) )
    {
        return 0;
    }
    else if ( ((act_psi.first < -kinematics_library::PI ) && (act_psi.first < kinematics_library::EPSILON) ) && 
              ((act_psi.second < kinematics_library::PI ) && (act_psi.second > kinematics_library::EPSILON)) )
    {
        act_psi.first = act_psi.first + (2*kinematics_library::PI );

        if(act_psi.first > act_psi.second)
            std::swap(act_psi.first, act_psi.second);

        return 0;
    }
    else if ( ((act_psi.first > -kinematics_library::PI ) && (act_psi.first < kinematics_library::EPSILON) ) &&
            ((act_psi.second > kinematics_library::PI ) && (act_psi.second > kinematics_library::EPSILON)) )
    {
        act_psi.second = act_psi.second - (2*kinematics_library::PI );

        if(act_psi.first > act_psi.second)
            std::swap(act_psi.first, act_psi.second);

        return 0;
    }
    else
    {
        return SRSKinematic::ERR_PSI_REARRANGE_RANGE;
    }
}

std::string SRSKinematicSolver::printError(int err)
{
    std::string error_message;

    switch(err)
    {
        case SRSKinematic::SUCCESS                    : error_message = " Kinematic success "; break;
        case SRSKinematic::ERR_BOUND_J1               : error_message = " Boundary condition failed in Joint 1 "; break;
        case SRSKinematic::ERR_BOUND_J2               : error_message = " Boundary condition failed in Joint 2 "; break;
        case SRSKinematic::ERR_BOUND_J3               : error_message = " Boundary condition failed in Joint 3 "; break;
        case SRSKinematic::ERR_BOUND_J5               : error_message = " Boundary condition failed in Joint 5 "; break;
        case SRSKinematic::ERR_BOUND_J6               : error_message = " Boundary condition failed in Joint 6 "; break;
        case SRSKinematic::ERR_BOUND_J7               : error_message = " Boundary condition failed in Joint 7 "; break;
        case SRSKinematic::ERR_REACH                  : error_message = " Pose not reachable "; break;
        case SRSKinematic::ERR_TRANS_EQU              : error_message = " Error in solving the transcendental equation - the sqaure root is complex "; break;
        case SRSKinematic::ERR_TRANS_EQU_COND         : error_message = " A condition arised while solving the transcendental equation - It need to be studied "; break;
        case SRSKinematic::ERR_TRANS_EQU_PICK_1       : error_message = " A condition (cond 1) arised while picking a solution from the result obatined through transcendental equation - It need to be studied "; break;
        case SRSKinematic::ERR_TRANS_EQU_PICK_2       : error_message = " A condition (cond 2) arised while picking a solution from the result obatined through transcendental equation - It need to be studied "; break;
        case SRSKinematic::ERR_TRANS_EQU_PICK_3       : error_message = " A condition (cond 3) arised while picking a solution from the result obatined through transcendental equation - It need to be studied "; break;
        case SRSKinematic::ERR_TRANS_EQU_PICK_4       : error_message = " A condition (cond 4) arised while picking a solution from the result obatined through transcendental equation - It need to be studied "; break;
        case SRSKinematic::ERR_TRANS_EQU_PICK_5       : error_message = " A condition (cond 5) arised while picking a solution from the result obatined through transcendental equation - It need to be studied "; break;
        case SRSKinematic::ERR_PSI_REARRANGE_RANGE    : error_message = " Error while rearranging the psi range - It need to be studied "; break;
        case SRSKinematic::ERR_UNION_SINGLE           : error_message = " Error in calculating the union of joints with only one feasible armangle "; break;
        case SRSKinematic::ERR_COMPLIMENT             : error_message = " Error in calculating the complement of infeasible arm angle "; break;
        case SRSKinematic::ERR_UNION_ALL              : error_message = " There is no intersection in the feasible armangles "; break;
        default                                       : error_message = " This error message is not defined";
    }

    return error_message;
}

void SRSKinematicSolver::save_joint_function(const double &an, const double &bn, const double &cn,
                                             const double &ad, const double &bd, const double &cd,
                                             const double &min_jtag, const double &max_jtag, const char *outputdata_file, 
                                             const std::string &type)
{
    FILE *fp, *fp_1;  // To save data
    double theta;

    std::stringstream filename;
    filename << srs_config_.save_psi_path<<"/"<<outputdata_file<<".dat";
    
    std::stringstream jtlimit_filename;
    jtlimit_filename << srs_config_.save_psi_path<<"/"<<outputdata_file<<"_jtlimit.dat";

    if (!(fp = fopen(filename.str().c_str(),"w")))
    {
        printf("Can't open file.\n");
        exit(1);
    }
    
    if (!(fp_1 = fopen(jtlimit_filename.str().c_str(),"w")))
    {
        printf("Can't open jtlimit_filename file.\n");
        exit(1);
    }

    for(double i = -kinematics_library::PI ; i<= kinematics_library::PI ; i=i+0.0174532925)
    {
        if(type.compare("TANGENT") == 0)
            theta = atan2( ( (an * sin(i))+(bn * cos(i))+cn ), ( (ad * sin(i))+(bd * cos(i))+cd ) );
        else if (type.compare("COSINE") == 0)
            theta = acos((an * sin(i)) + (bn * cos(i)) + cn);
        else 
            theta = 0;

        fprintf(fp,"%f %f \n", (i*kinematics_library::RTD), (theta*kinematics_library::RTD));
        fprintf(fp_1,"%f %f \n", (i*kinematics_library::RTD), (min_jtag*kinematics_library::RTD));
        fprintf(fp_1,"%f %f \n", (i*kinematics_library::RTD), (max_jtag*kinematics_library::RTD));
    }

    fclose(fp);
    fclose(fp_1);
}

void SRSKinematicSolver::save_psi_file()
{

    for(std::size_t i = 0; i< feasible_psi.size(); i++)
    {
        if(feasible_psi.at(i).joint_number == 1)                        
            save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, jts_limits_[0].first, jts_limits_[0].second);
        else if(feasible_psi.at(i).joint_number == 2)
            save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, jts_limits_[1].first, jts_limits_[1].second);
        else if(feasible_psi.at(i).joint_number == 3)
            save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, jts_limits_[2].first, jts_limits_[2].second);
        else if(feasible_psi.at(i).joint_number == 5)
            save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, jts_limits_[4].first, jts_limits_[4].second);
        else if(feasible_psi.at(i).joint_number == 6)
            save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, jts_limits_[5].first, jts_limits_[5].second);
        else if(feasible_psi.at(i).joint_number == 7)
            save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, jts_limits_[6].first, jts_limits_[6].second);
    }

    for(std::size_t i = 0; i< infeasible_psi.size(); i++)
    {
        if(infeasible_psi.at(i).joint_number == 1)
            save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, jts_limits_[0].first, jts_limits_[0].second);
        else if(infeasible_psi.at(i).joint_number == 2)
            save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, jts_limits_[1].first, jts_limits_[1].second);
        else if(infeasible_psi.at(i).joint_number == 3)
            save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, jts_limits_[2].first, jts_limits_[2].second);
        else if(infeasible_psi.at(i).joint_number == 5)
            save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, jts_limits_[4].first, jts_limits_[4].second);
        else if(infeasible_psi.at(i).joint_number == 6)
            save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, jts_limits_[5].first, jts_limits_[5].second);
        else if(infeasible_psi.at(i).joint_number == 7)
            save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, jts_limits_[6].first, jts_limits_[6].second);
    }
}

void SRSKinematicSolver::save_psi_file_helper(const char* outputdata_file, double min, double max, double min_jt, double max_jt)
{
    FILE *fp;
    std::stringstream psi_filename;
    psi_filename << srs_config_.save_psi_path<<"/"<<outputdata_file<<"_psi.dat";
    
    if (!(fp = fopen(psi_filename.str().c_str(),"w")))
    {
        printf("Can't open psi_filename file.\n");
        exit(1);
    }

    fprintf(fp,"%f %f \n", min*kinematics_library::RTD, min_jt*kinematics_library::RTD);
    fprintf(fp,"%f %f \n", min*kinematics_library::RTD, max_jt*kinematics_library::RTD);
    fprintf(fp,"%f %f \n", max*kinematics_library::RTD, max_jt*kinematics_library::RTD);
    fprintf(fp,"%f %f \n", max*kinematics_library::RTD, min_jt*kinematics_library::RTD);
    fclose(fp);
}

}
