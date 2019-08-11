#include <solver/SRSKinematicSolver.hpp>

namespace kinematics_library
{

SRSKinematicSolver::SRSKinematicSolver ( const KinematicsConfig &kinematics_config,  const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree,                                          
                                         const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain )
{

    kdl_tree_       = kdl_tree;
    kdl_chain_      = kdl_chain;

    fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_kinematic_chain );

    assignVariables ( kinematics_config, kdl_chain_ );
    kdl_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    kdl_ik_jt_array_.resize ( kdl_chain_.getNrOfJoints() );

}

SRSKinematicSolver::~SRSKinematicSolver()
{
    if ( fk_kdlsolver_pos_ ) 
    {
        fk_kdlsolver_pos_ = NULL;
        delete fk_kdlsolver_pos_;
    }
}

bool SRSKinematicSolver::solveIK (const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                            std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status )
{
    convertPoseBetweenDifferentFrames ( kdl_tree_, target_pose, kinematic_pose_ );
    
    getKinematicJoints ( kdl_chain_, joint_status, jt_names_, current_jt_status_ );

    
}

bool SRSKinematicSolver::solveFK (const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status )
{
    getKinematicJoints ( kdl_chain_, joint_angles, jt_names_, current_jt_status_ );

    convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );

    if ( fk_kdlsolver_pos_->JntToCart ( kdl_jt_array_, kdl_frame_ ) >= 0 ) 
    {
        kdlToRbs ( kdl_frame_, kinematic_pose_ );
        solver_status.statuscode = KinematicsStatus::FK_FOUND;
        convertPoseBetweenDifferentFrames ( kdl_tree_, kinematic_pose_, fk_pose );
        return true;
    }

    solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
    return false;
}



int SRSKinematicSolver::invkin(const double pos[3], const double rot[3], double jointangles[7])
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

    Eul2RotMat(rot,Rd);  //eul_zyx
    Mult_mat_vec(Rd,l_wt,t_Xsw);

    Xsw.at(0) = pos[0]-l_bs.at(0)-t_Xsw.at(0);
    Xsw.at(1) = pos[1]-l_bs.at(1)-t_Xsw.at(1);
    Xsw.at(2) = pos[2]-l_bs.at(2)-t_Xsw.at(2);


    t_thet4=((Xsw.at(0)*Xsw.at(0))+(Xsw.at(1)*Xsw.at(1))+(Xsw.at(2)*Xsw.at(2)));

    if (sqrt(t_thet4) >(DSE+DEW))
    {
    succeeded = SRSKinematic::ERR_REACH;
    }
    else
    {
        thet4=((t_thet4 - (DSE * DSE) - (DEW * DEW)) / (2.0 * DSE * DEW) );

        jointangles[3] = -acos(thet4);  //using cosine law


        if(DEBUG)
        std::cout<< "JOINT 4 = "<<jointangles[3]*SRSKinematic::RTD<<"   "<<thet4<<"  "<<sqrt(1-(thet4*thet4))<<std::endl;

        //Below calculate the reference shoulder angle
        rot_matrix(jointangles[3], SRSKinematic::PI /2.0, R34);

        Mult_mat_vec(R34, l_ew, t_R34);

        t_refthe2.at(0) = l_se.at(0) + t_R34.at(0);
        t_refthe2.at(1) = l_se.at(1) + t_R34.at(1);
        t_refthe2.at(2) = l_se.at(2) + t_R34.at(2);

        the2_R=atan2(t_refthe2.at(0),t_refthe2.at(1)) - atan2(sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))),-Xsw.at(2));
        //the2_R=atan2(-t_refthe2.at(0),-t_refthe2.at(1)) - atan2(sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))),Xsw.at(2));
        //the2_R = 2*atan( (t_refthe2.at(0) + ( sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))) ) ) / (Xsw.at(2) - t_refthe2.at(0)) );
        //the2_R = the2_R +(SRSKinematic::PI );

        c2 = cos(the2_R);
        s2 = sin(the2_R);

        the1_R_var1 = (c2 * t_refthe2.at(0)) - (s2 * t_refthe2.at(1));

        c1 = Xsw.at(0) / the1_R_var1;
        s1 = Xsw.at(1) / the1_R_var1;

        the1_R=atan2(s1,c1);

        if(DEBUG)
        {
        std::cout<< "the1_R = "<<the1_R*SRSKinematic::RTD<<"  the2_R = "<<the2_R*SRSKinematic::RTD<<std::endl;
        std::cout<<"t_refthe2= "<<t_refthe2.at(0)<<"  "<<t_refthe2[1]<<"  "<<t_refthe2.at(2)<<"  "<<(t_refthe2.at(0)+Xsw.at(2))<<"  "<<((t_refthe2.at(1)*t_refthe2.at(1))-(t_refthe2.at(0)*t_refthe2.at(0))-(Xsw.at(2)*Xsw.at(2))) <<"  "<<std::endl;
        std::cout<< "atan2 ="<<atan2(t_refthe2.at(0),t_refthe2.at(1))*SRSKinematic::RTD<<"   "<<"next= "<<atan2(sqrt((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2))),-Xsw.at(2))*SRSKinematic::RTD<<"  "<<((t_refthe2.at(0)*t_refthe2.at(0))+(t_refthe2.at(1)*t_refthe2.at(1))-(Xsw.at(2)*Xsw.at(2)))<<"  "<<Xsw.at(2)<<std::endl;
        }

        rot_matrix(the1_R, -SRSKinematic::PI /2.0, R01);
        rot_matrix(the2_R, SRSKinematic::PI /2.0, R12);
        rot_matrix(0.0     , -SRSKinematic::PI /2.0, R23);
        Mult_mat_mat(R01, R12, t_r03_r);
        Mult_mat_mat(t_r03_r, R23, R_03_R);

        m_Xsw=sqrt( (Xsw.at(0) * Xsw.at(0)) + (Xsw.at(1) * Xsw.at(1)) + (Xsw.at(2) * Xsw.at(2)) );
        usw.at(0)=Xsw.at(0) / m_Xsw;
        usw.at(1)=Xsw.at(1) / m_Xsw;
        usw.at(2)=Xsw.at(2) / m_Xsw;

        usw_sk.at(0) =  0.0;            usw_sk.at(3) = -usw.at(2);      usw_sk.at(6) =  usw.at(1);
        usw_sk.at(1) =  usw.at(2);      usw_sk.at(4) = 0.0;             usw_sk.at(7) = -usw.at(0);
        usw_sk.at(2) = -usw.at(1);      usw_sk.at(5) = usw.at(0);       usw_sk.at(8) = 0.0;

        //Shoulder joint
        Mult_mat_mat(usw_sk, R_03_R, As);
        Mult_mat_mat(usw_sk, usw_sk, t_Bs);

        for(int i=0; i<9; i++)
        t_Bs.at(i) = -1.0 * t_Bs.at(i);

        Mult_mat_mat(t_Bs, R_03_R, Bs);
        Mult_vec_tslvec(usw, t_Cs);                        
        Mult_mat_mat(t_Cs, R_03_R, Cs);

        std::cout<<std::endl;
        for(int i = 0; i < 3; i++)
            std::cout<<R34.at(i)<<"  "<<R34.at(i+3)<<"  "<<R34.at(i+6)<<std::endl;
        std::cout<<std::endl;

        //Wrist Joint
        trans_mat(R34, tsl_R34);
        trans_mat(As, tsl_As);
        trans_mat(Bs, tsl_Bs);
        trans_mat(Cs, tsl_Cs);

        Mult_mat_mat(tsl_R34, tsl_As, t_Aw);
        Mult_mat_mat(t_Aw, Rd, Aw);
        Mult_mat_mat(tsl_R34, tsl_Bs, t_Bw);
        Mult_mat_mat(t_Bw, Rd, Bw);
        Mult_mat_mat(tsl_R34, tsl_Cs, t_Cw);
        Mult_mat_mat(t_Cw, Rd, Cw);

        if (DEBUG)
        {
            std::cout<<"  Xsw "<<std::endl;
            std::cout<<Xsw.at(0)<<"  "<<Xsw.at(1)<<"  "<<Xsw.at(2)<<std::endl;

            std::cout<<"  usw "<<std::endl;
            std::cout<<usw.at(0)<<"  "<<usw.at(1)<<"  "<<usw.at(2)<<std::endl;

            std::cout<<"m_Xww = "<<m_Xsw<<std::endl;

            std::cout<<"the1_R = "<<the1_R*SRSKinematic::RTD<<"   the2_R="<<the2_R*SRSKinematic::RTD<<std::endl;

            std::cout<<"  R01 "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<R01.at(i)<<"  "<<R01.at(i+3)<<"  "<<R01.at(i+6)<<std::endl;

            std::cout<<"  R12 "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<R12.at(i)<<"  "<<R12.at(i+3)<<"  "<<R12.at(i+6)<<std::endl;

            std::cout<<"  R23 "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<R23.at(i)<<"  "<<R23.at(i+3)<<"  "<<R23.at(i+6)<<std::endl;

            std::cout<<"  R_03_R "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<R_03_R.at(i)<<"  "<<R_03_R.at(i+3)<<"  "<<R_03_R.at(i+6)<<std::endl;

            std::cout<<"  AS "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<As.at(i)<<"  "<<As.at(i+3)<<"  "<<As.at(i+6)<<std::endl;

            std::cout<<"  BS "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<Bs.at(i)<<"  "<<Bs.at(i+3)<<"  "<<Bs.at(i+6)<<std::endl;


            std::cout<<"  CS "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<Cs.at(i)<<"  "<<Cs.at(i+3)<<"  "<<Cs.at(i+6)<<std::endl;


            std::cout<<"  Aw "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<Aw.at(i)<<"  "<<Aw.at(i+3)<<"  "<<Aw.at(i+6)<<std::endl;


            std::cout<<"  bw "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<Bw.at(i)<<"  "<<Bw.at(i+3)<<"  "<<Bw.at(i+6)<<std::endl;


            std::cout<<"  Cw "<<std::endl;
            for(int i = 0; i < 3; i++)
            std::cout<<Cw.at(i)<<"  "<<Cw.at(i+3)<<"  "<<Cw.at(i+6)<<std::endl;


            std::cout<<"-----------------------------------------------------------"<<std::endl;
        }


        if(DEBUG)
        {
            // joint 1
            save_tangent_joint_function(-As[4], -Bs[4], -Cs[4], -As[3], -Bs[3], -Cs[3],min_j1, max_j1, "joint1");
            // joint 2
            save_cosine_joint_function(-As[5], -Bs[5], -Cs[5], min_j2, max_j2, "joint2");                                    
            // joint 3
            save_tangent_joint_function(As[8], +Bs[8], +Cs[8], -As[2], -Bs[2], -Cs[2], min_j3, max_j3, "joint3");
            // joint 5
            save_tangent_joint_function(+Aw[7], +Bw[7], +Cw[7], +Aw[6], +Bw[6], +Cw[6], min_j5, max_j5, "joint5");                                    
            // joint 6
            save_cosine_joint_function(Aw[8], Bw[8], Cw[8],min_j6, max_j6, "joint6");
            // joint 7
            save_tangent_joint_function(+Aw[5], +Bw[5], +Cw[5], -Aw[2], -Bw[2], -Cw[2], min_j7, max_j7, "joint7");
        }

        //Calculating Arm Angle
        armAngle_result = cal_armangle(As,Bs,Cs,Aw,Bw,Cw,final_feasible_armangle);

        if (armAngle_result==0)
        {
            if(DEBUG)
                save_psi_file();

            AA=final_feasible_armangle.at(0).first;

            jointangles[0]=atan2((-(As[4]*sin(AA))-(Bs[4]*cos(AA))-(Cs[4])),(-(As.at(3)*sin(AA))-(Bs.at(3)*cos(AA))-(Cs.at(3))));
            jointangles[1]=acos(-(As[5]*sin(AA))-(Bs[5]*cos(AA))-(Cs[5]));
            jointangles[2]=atan2(((As[8]*sin(AA))+(Bs[8]*cos(AA))+(Cs[8])),(-(As.at(2)*sin(AA))-(Bs.at(2)*cos(AA))-(Cs.at(2))));
            jointangles[4]=atan2(((Aw[7]*sin(AA))+(Bw[7]*cos(AA))+(Cw[7])),((Aw[6]*sin(AA))+(Bw[6]*cos(AA))+(Cw[6])));
            jointangles[5]=acos(((Aw[8]*sin(AA))+(Bw[8]*cos(AA))+(Cw[8])));
            jointangles[6]=atan2(((Aw[5]*sin(AA))+(Bw[5]*cos(AA))+(Cw[5])),(-(Aw.at(2)*sin(AA))-(Bw.at(2)*cos(AA))-(Cw.at(2))));
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

    succeeded = tangenttype_armangle(-As[4], -Bs[4], -Cs[4], -As[3], -Bs[3], -Cs[3], at1, bt1, ct1, cond1, 1, min_j1, max_j1, "joint1");

    // if (succeeded != 0)
    //        return succeeded;

    //Joint 2
    if(DEBUG)
    {
        std::cout<<"-----------------------------------"<<std::endl;
        std::cout<<"im in joint 2"<<std::endl;
        std::cout<<std::endl;
    }

    succeeded = feasible_armangle_cosinetype_cyclicfunction(-As[5], -Bs[5], -Cs[5], min_j2, max_j2, 2, "joint2");

    //if (succeeded != 0)
    //       return succeeded;

    //Joint 3

    succeeded = tangenttype_armangle(As[8], +Bs[8], +Cs[8], -As[2], -Bs[2], -Cs[2], at3, bt3, ct3, cond3, 3, min_j3, max_j3, "joint3");

    //if (succeeded != 0)
    //        return succeeded;

    //Joint 5

    succeeded = tangenttype_armangle(+Aw[7], +Bw[7], +Cw[7], +Aw[6], +Bw[6], +Cw[6], at5, bt5, ct5, cond5, 5, min_j5, max_j5, "joint5");

    //if (succeeded != 0)
    //        return succeeded;

    //Joint 6
    if(DEBUG)
    {
        std::cout<<"-----------------------------------"<<std::endl;
        std::cout<<"im in joint 6"<<std::endl;
        std::cout<<std::endl;
    }

    succeeded = feasible_armangle_cosinetype_cyclicfunction(Aw[8], Bw[8], Cw[8], min_j6, max_j6, 6, "joint6");

    if (succeeded != 0)
        return succeeded;

    //Joint 7
    succeeded = tangenttype_armangle(+Aw[5], +Bw[5], +Cw[5], -Aw[2], -Bw[2], -Cw[2], at7, bt7, ct7, cond7, 7, min_j7, max_j7, "joint7");

    // if (succeeded != 0)
    //        return succeeded;


    // debugging
    if(DEBUG)
    {
        std::cout<<"Feasible Psi "<<std::endl;
        for(int i = 0; i< feasible_psi.size(); i++)
        {
            for(int j = 0; j< feasible_psi.at(i).psi.size(); j++)
                std::cout<<"Joint "<<feasible_psi.at(i).joint_number<<"  "<<feasible_psi.at(i).joint_name<<"  "<< "Psi = "<<feasible_psi.at(i).psi.at(j).first*SRSKinematic::RTD<<"  "<<feasible_psi.at(i).psi.at(j).second*SRSKinematic::RTD<<std::endl;
        }

        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"Infeasible Psi "<<std::endl;
        for(int i = 0; i< infeasible_psi.size(); i++)
        {                        
            for(int j = 0; j< infeasible_psi.at(i).psi.size(); j++)
                std::cout<<"Joint "<<infeasible_psi.at(i).joint_number<<"  "<<infeasible_psi.at(i).joint_name<<"  "<< "Psi = "<<infeasible_psi.at(i).psi.at(j).first*SRSKinematic::RTD<<"  "<<infeasible_psi.at(i).psi.at(j).second*SRSKinematic::RTD<<std::endl;
        }
        std::cout<<std::endl;
        std::cout<<std::endl;
    }

    //std::pair<double, double> single_feasible_result;
    std::vector< ArmAngle > single_feasible_result;

    int intesection_result = union_joints_with_only_one_feasible_armangle(feasible_psi, single_feasible_result);

    if(DEBUG)
    {
        std::cout<<"intersected Feasible Psi "<<std::endl;

        for(int i = 0; i< single_feasible_result.size(); i++)
        {
            for(int j = 0; j< single_feasible_result.at(i).psi.size(); j++)
                std::cout<<"Joint "<<single_feasible_result.at(i).joint_number<<"  "<<single_feasible_result.at(i).joint_name<<"    "<< single_feasible_result.at(i).psi.at(j).first*SRSKinematic::RTD<<"  "<<single_feasible_result.at(i).psi.at(j).second*SRSKinematic::RTD<<std::endl;
        }
    }


    complement_of_infeasible_psi(infeasible_psi, single_feasible_result);

    succeeded = union_of_all_feasible_armangle(single_feasible_result, final_feasible_armangle);
    if( succeeded != 0)
        return succeeded;


    if(DEBUG)
    {

        std::cout<<"compliemnt infeasbile psi"<<std::endl;

        for(int i = 0; i< single_feasible_result.size(); i++)
        {
            for(int j = 0; j< single_feasible_result.at(i).psi.size(); j++)
                std::cout<<"Joint "<<single_feasible_result.at(i).joint_number<<"  "<<single_feasible_result.at(i).joint_name<<"    "<< single_feasible_result.at(i).psi.at(j).first*SRSKinematic::RTD<<"  "<<single_feasible_result.at(i).psi.at(j).second*SRSKinematic::RTD<<std::endl;
        }

        std::cout<<"Final feasbile psi "<<final_feasible_armangle.size()<< std::endl;

        for(int i = 0; i< final_feasible_armangle.size(); i++)
        {
            std::cout<<final_feasible_armangle.at(i).first*SRSKinematic::RTD<<"  "<<final_feasible_armangle.at(i).second*SRSKinematic::RTD<<std::endl;
        }
    }

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

    if(DEBUG)
    {
        std::cout<<"-----------------------------------"<<std::endl;
        std::cout<<"im in joint = "<<jointnumber<<" cond = "<<condition<<std::endl;
        std::cout<<std::endl;
    }

    if ((condition < SRSKinematic::PAC) && (condition > SRSKinematic::NAC))
    {
        if(DEBUG)
            std::cout<<"Cond 1"<<std::endl;

        succeeded = feasible_armangle_tangenttype_stationarycase(an, bn, cn, ad, bd, cd, at, bt, ct, jointnumber, jointname);

        if (succeeded != 0)
            return succeeded;
    }
    else if (condition > 0)
    {
        if(DEBUG)
            std::cout<<"Cond 2"<<std::endl;

        succeeded = feasible_armangle_tangenttype_cyclicfunction(an, bn, cn, ad, bd, cd, at, bt, ct, min_jointlimit, max_jointlimit, jointnumber, jointname);

        if (succeeded != 0)
            return succeeded;

    }
    else if (condition < 0)
    {
        if(DEBUG)
        std::cout<<"Cond 3"<<std::endl;

        succeeded = feasible_armangle_monotonicfunction(an, bn, cn, ad, bd, cd, min_jointlimit, max_jointlimit, jointnumber, jointname);

        if (succeeded != 0)
            return succeeded;
    }

}

int SRSKinematicSolver::feasible_armangle_tangenttype_stationarycase(const double &an, const double &bn, const double &cn,
                                                                    const double &ad, const double &bd, const double &cd,
                                                                    const double &at, const double &bt, const double &ct,
                                                                    const int &jointnumber, const std::string& jointname)
{
    double psi_stationary = 0.0;
    double left_jtlimit = 0.0, right_jtlimit = 0.0;
    ArmAngle calculated_psi;


    psi_stationary = 2 * atan( at / (bt - ct) );

    calculated_psi.joint_name = jointname;
    calculated_psi.joint_number = jointnumber;
    calculated_psi.psi.resize(1);
    calculated_psi.psi.at(0) = std::make_pair(psi_stationary,psi_stationary);

    infeasible_psi.push_back(calculated_psi);

    //infeasible_psi.push_back(std::make_pair(psi_stationary,psi_stationary));
    //todo: find the use of this limit

    left_jtlimit    = atan2( (( (at * bn) - (bt * an)) / -ct), ( ((at * bd) - (bt * ad)) / -ct ) );
    right_jtlimit   = atan2( (( (at * bn) - (bt * an)) /  ct), ( ((at * bd) - (bt * ad)) /  ct ) );

    if(DEBUG)
    {
        std::cout<<"        SINGULAR CASE               "<<std::endl;
        std::cout<<"psi_stationary = "<<psi_stationary*SRSKinematic::RTD<<std::endl;
        //std::cout<<"global_min = "<<global_min*SRSKinematic::RTD<<"  global_max = "<<global_max*SRSKinematic::RTD<<std::endl;
        //std::cout<<"psi.first = "<<psi.first*SRSKinematic::RTD<<"  psi.second = "<<psi.second*SRSKinematic::RTD<<std::endl;
    }

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

    if(DEBUG)
    {
        std::cout<<std::endl;
        std::cout<<"   MONOTONIC "<<std::endl;
        std::cout<<std::endl;
    }

    // +90 and -90 can be made in to one if loop
    if (fabs(min_jtag - SRSKinematic::PI /2.0) < ZERO) //+90
    {
        a = bd;
        b = ad;
        c = -cd;
    }
    else if (fabs(min_jtag + SRSKinematic::PI /2.0) < ZERO) //-90
    {
        a = -bd;
        b = -ad;
        c = cd;
    }
    else if ((fabs(min_jtag - SRSKinematic::PI ) < ZERO) || (fabs(min_jtag + SRSKinematic::PI ) < ZERO) ) //180|| -180
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

    if(DEBUG)
        std::cout<<a<<"  "<<b<<"  "<<c<<std::endl;

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
    if(DEBUG)
    {                        
        std::cout<<"calculated_psi.psi.first = "<<calculated_psi.psi.at(0).first*SRSKinematic::RTD<<std::endl;
    }

    if ( fabs(max_jtag - SRSKinematic::PI /2.0) < ZERO) //+90
    {
        a = bd;
        b = ad;
        c = -cd;
    }
    else if (fabs(max_jtag + SRSKinematic::PI /2.0) < ZERO) //-90
    {
        a = -bd;
        b = -ad;
        c = cd;
    }
    else if ((fabs(max_jtag - SRSKinematic::PI ) < ZERO) || (fabs(max_jtag + SRSKinematic::PI ) < ZERO) ) //180|| -180
    {
        a =  bn;
        b =  an;
        c = -cn;
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

    if(DEBUG)
        std::cout<<a<<"  "<<b<<"  "<<c<<std::endl;

    succeeded = solve_transcendental_equation(a, b, c, transcendental_solution);
    if( succeeded != 0)
        return succeeded;

    succeeded = psi_picker(transcendental_solution, "MAX", calculated_psi.psi.at(0).second);
    //succeeded = solve_transcendental_equation(a, b, c, calculated_psi.psi.at(0));

    if( succeeded != 0 )
        return succeeded;

    feasible_psi.push_back(calculated_psi);

    if(DEBUG)
    {
        std::cout<<"calculated_psi.psi.second = "<<calculated_psi.psi.at(0).second*SRSKinematic::RTD<<std::endl;
    }

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


    if(DEBUG)
    {
        std::cout<<std::endl;
        std::cout<<"   COSINE CYCLIC "<<std::endl;
        std::cout<<std::endl;
    }

    t_sqrt = sqrt((at * at) + (bt * bt));

    if(fabs(at) <= ZERO)
    {
        psi_min = SRSKinematic::PI ;
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

    std::cout<<"global_min = "<<global_min*SRSKinematic::RTD<<"  global_max = "<<global_max*SRSKinematic::RTD<<std::endl;

    if(global_min > global_max)
    {
        std::swap(global_min, global_max);
        swap_flag = true;

        if(DEBUG)
        {
            std::cout<<std::endl;
            std::cout<<std::endl;
            std::cout<< "***************************************   swaping ************************************"<<"global_min "<<global_min*SRSKinematic::RTD<<" "<<"global_max "<<global_max*SRSKinematic::RTD<<std::endl;
            std::cout<<std::endl;
            std::cout<<std::endl;
        }
    }

    if(DEBUG)
    {
        std::cout<< t_sqrt<<"  "<<at<<"  "<<bt<<"  "<<ct<<"  "<<ZERO<<std::endl;
        std::cout<<( (at * at) + (bt * bt) - ((ct-1.0) * (ct-1.0)) ) <<std::endl;
        std::cout<<( (at * at) + (bt * bt) - ((ct+1.0) * (ct+1.0)) )<<std::endl;
        std::cout<<"global_min = "<<global_min*SRSKinematic::RTD<<"  global_max = "<<global_max*SRSKinematic::RTD<<std::endl;
        std::cout<<"psi_min = "<<psi_min*SRSKinematic::RTD<<"  psi_max = "<<psi_max*SRSKinematic::RTD<<std::endl;
    }

    calculated_psi.joint_name = jointname;
    calculated_psi.joint_number = jointnumber;

    //if ((fabs( (at * at) + (bt * bt) - ((ct-1) * (ct-1)) )) < ZERO)
    if ( (((at * at) + (bt * bt) - ((ct-1) * (ct-1)) ) <SRSKinematic::PAC ) && (((at * at) + (bt * bt) - ((ct-1) * (ct-1)) ) >SRSKinematic::NAC))
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
    else if (fabs(( (at * at) + (bt * bt) - ((ct+1) * (ct+1)) )) < ZERO)
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
        if( (global_min > max_jtag) || (global_max < min_jtag) )                                                //cond-1
        {
            if(DEBUG)
                std::cout<< "cond-1"<<std::endl;

            succeeded = -(jointnumber);
            if(DEBUG)
            {
                std::cout<< "   Failed in boundary condition in cosine function    "<<std::endl;
                std::cout<<"psi_min = "<<psi_min*SRSKinematic::RTD<<"  psi_max = "<<psi_max*SRSKinematic::RTD<<std::endl;
                std::cout<<"global_min = "<<global_min*SRSKinematic::RTD<<"  global_max = "<<global_max*SRSKinematic::RTD<<std::endl;
                std::cout<<"psi.first = "<<psi.first*SRSKinematic::RTD<<"  psi.second = "<<psi.second*SRSKinematic::RTD<<std::endl;
            }

            return succeeded;
        }
        else if( (global_min < min_jtag) && (( min_jtag <= global_max) && (global_max <= max_jtag)) )           //cond-2
        {
            if(DEBUG)
            std::cout<< "cond-2"<<std::endl;

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
        else if( ((min_jtag <= global_min) && (global_min <= max_jtag)) && (global_max >= max_jtag) )           //cond-3                                
        {

            // solve equation 24
            c = cos(max_jtag) - ct;

            if(DEBUG)
            {
                std::cout<< "cond-3"<<std::endl;
                std::cout<<"a = "<< a <<"   "<<"b = "<<b<<"  "<<"c = "<<c<<std::endl;
            }

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
        else if( (global_min < min_jtag) && (global_max > max_jtag ) )                                          //cond-4
        {
            if(DEBUG)
            std::cout<< "cond-4"<<std::endl;

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
        else if( ((min_jtag <= global_min) && (global_min <= max_jtag )) &&
                ((min_jtag <= global_max) && (global_max <= max_jtag )) )                                      //cond-5
        {
            if(DEBUG)
            std::cout<< "cond-5"<<std::endl;

            calculated_psi.psi.resize(1);
            calculated_psi.psi.at(0) = std::make_pair(-SRSKinematic::PI , SRSKinematic::PI );

            feasible_psi.push_back(calculated_psi);

        }                        

    }

    if(DEBUG)
    {
        std::cout<<"at = "<< at <<"   "<<"bt = "<<bt<<"  "<<"ct = "<<ct<<std::endl;
        std::cout<<"cond 1 = "<<( (at * at) + (bt * bt) - ((ct-1) * (ct-1)) )<<"  "<<"cond 2 = "<<( (at * at) + (bt * bt) - ((ct+1) * (ct+1)) )<<std::endl;
        std::cout<<"t_sqrt = "<< t_sqrt <<"   "<<"squared = "<<((at * at) + (bt * bt))<<std::endl;
        std::cout<<"psi_min = "<<psi_min*SRSKinematic::RTD<<"  psi_max = "<<psi_max*SRSKinematic::RTD<<std::endl;
    }

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

    if(DEBUG)
    {
            std::cout<<std::endl;
            std::cout<<"   TANGENT CYCLIC "<<std::endl;
            std::cout<<std::endl;
    }

    t_sqrt = sqrt( (at * at) + (bt * bt) - (ct * ct) );
    psi_min = 2.0*(atan( (at-t_sqrt) / (bt-ct)) );
    psi_max = 2.0*(atan( (at+t_sqrt) / (bt-ct)) );

    if(DEBUG)
            std::cout<<"psi_min "<<psi_min*SRSKinematic::RTD<<" "<<"psi_max "<<psi_max*SRSKinematic::RTD<<std::endl;



    global_min = atan2( ( (an * sin(psi_min))+(bn * cos(psi_min))+cn ), ( (ad * sin(psi_min))+(bd * cos(psi_min))+cd ) );
    global_max = atan2( ( (an * sin(psi_max))+(bn * cos(psi_max))+cn ), ( (ad * sin(psi_max))+(bd * cos(psi_max))+cd ) );

    /*if(global_min > global_max)
    {
            swap(global_min, global_max);

            if(DEBUG)
                    std::cout<< "*************   swaping **********"<<"global_min "<<global_min*SRSKinematic::RTD<<" "<<"global_max "<<global_max*SRSKinematic::RTD<<std::endl;

    }*/


    if(DEBUG)
            std::cout<< "Global min "<<global_min*SRSKinematic::RTD<<" "<<"global_max "<<global_max*SRSKinematic::RTD<<std::endl;


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
        if(DEBUG)
                std::cout<< "cond-2"<<std::endl;

        calculated_psi.psi.resize(1);
        //calculated_psi.psi.at(0) = std::make_pair(0.0, 0.0);

        succeeded = calculate_region_armAngle_tangentcylic(an, bn, cn, ad, bd, cd, min_jtag, calculated_psi.psi.at(0) );
        if( succeeded != 0)
                return succeeded;


        infeasible_psi.push_back(calculated_psi);
        //std::cout<< "c2"<<std::endl;

    }
    else if( ((min_jtag <= global_min) && (global_min <= max_jtag)) && (global_max >= max_jtag) )           //cond-3
    {
        if(DEBUG)
                std::cout<< "cond-3"<<std::endl;

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
        //std::cout<< "c3"<<std::endl;

    }
    else if( (global_min < min_jtag) && (global_max > max_jtag ) )                                          //cond-4
    {
        if(DEBUG)
                std::cout<< "cond-4"<<std::endl;

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
        //std::cout<< "c4"<<std::endl;

    }
    else if( ((min_jtag <= global_min) && (global_min <= max_jtag )) && 
             ((min_jtag <= global_max) && (global_max <= max_jtag )) )                          //cond-5
    {
        calculated_psi.psi.resize(1);
        calculated_psi.psi.at(0) = std::make_pair(-SRSKinematic::PI , SRSKinematic::PI );

        feasible_psi.push_back(calculated_psi);

    }

    if(DEBUG)
    {
        std::cout<<"psi_min = "<<psi_min*SRSKinematic::RTD<<"  psi_max = "<<psi_max*SRSKinematic::RTD<<std::endl;
        std::cout<<"global_min = "<<global_min*SRSKinematic::RTD<<"  global_max = "<<global_max*SRSKinematic::RTD<<std::endl;                       
    }

    std::cout<<" im joint "<<succeeded<<std::endl;

    return succeeded;

}

int SRSKinematicSolver::calculate_region_armAngle_tangentcylic(const double &an, const double &bn, const double &cn,
                                                            const double &ad, const double &bd, const double &cd,
                                                            const double &joint_limit, std::pair< double,double > &psi_pair )
{
    double a = 0.0, b = 0.0, c = 0.0;
    std::pair<double, double> psi(0.0, 0.0);
    int succeeded = 0;

    std::cout<<"joint_limit = "<<joint_limit*SRSKinematic::RTD<<std::endl;

    // solve equation 23
    // since tan(90) is infinity, the equ 23 is modified
    if (fabs(joint_limit - SRSKinematic::PI /2.0) < ZERO) //+90
    {
        a = bd;
        b = ad;
        c = -cd;
    }
    else if (fabs(joint_limit + SRSKinematic::PI /2.0) < ZERO) //-90
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

    if(DEBUG)
        std::cout<<"psi.first = "<<psi.first*SRSKinematic::RTD<<"  psi.second = "<<psi.second*SRSKinematic::RTD<<std::endl;

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
                    succeeded = SRSKinematic::ERR_TRANS_EQU;
            }
    }*/

    if ( fabs(b) < ZERO) //~0
    {
        std::cout<< "transcendental_solutions b is zero"<<std::endl;
        double temp = c/a;
        //transcendental_solutions.first = atan2(temp,-sqrt(1-(temp*temp)));
        //transcendental_solutions.second = atan2(temp,sqrt(1-(temp*temp)));

        transcendental_solutions.first = -acos(temp);
        transcendental_solutions.second = acos(temp);

        std::cout<< "transc_equ b zero "<<sqrt(fabs(1-(temp*temp)))<<"  "<<temp<<"  "<<atan2(temp,sqrt(1-(temp*temp)))*SRSKinematic::RTD<<std::endl;

    }
    else if ( fabs(c) < ZERO) //~0
    {
        transcendental_solutions.first = atan2(a,-b);
        transcendental_solutions.second = atan2(-a,b);

        if(DEBUG)
            std::cout<< "transcendental_solutions ' c<0 ' condition  "<<transcendental_solutions.first*SRSKinematic::RTD<<"   "<<transcendental_solutions.second*SRSKinematic::RTD<<"  "<<std::endl;
        succeeded =  SRSKinematic::ERR_TRANS_EQU_COND;

    }
    else
    {
        psi_1 = atan2(b,a);
        psi_2 = atan2(sqrt(squared_value), c);

        transcendental_solutions.first  = psi_1 - psi_2;
        transcendental_solutions.second = psi_1 + psi_2;
    }
    if(DEBUG)
    {
        std::cout<< "transc_equ  "<<psi_1*SRSKinematic::RTD<<"   "<<psi_2*SRSKinematic::RTD<<"  "<<sqrt(((a * a) + (b * b) - (c * c)) )<<"  "<<((a * a) + (b * b) - (c * c) )<<"  "<<"a ="<<a<<"  b="<<b<<"  c="<<c<<std::endl;
        std::cout<< "transc_equ solu "<<transcendental_solutions.first*SRSKinematic::RTD<<"   "<<transcendental_solutions.second*SRSKinematic::RTD<<std::endl;
    }
    
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
        transcendental_solutions.first  = SRSKinematic::PI + alpha;
        transcendental_solutions.second = SRSKinematic::PI + alpha;
    }
    else
    {
        transcendental_solutions.first  = acos(temp_cos) + alpha;
        transcendental_solutions.second = acos(temp_cos) + alpha;
    }
    if(DEBUG)
    {
        std::cout<< "new transc_equ  "<<R<<"  "<<sin_alpha<<"   "<<cos_alpha<<"   "<<alpha*SRSKinematic::RTD<<"  "<<temp_cos<<"  "<<"a ="<<a<<"  b="<<b<<"  c="<<c<<std::endl;
        std::cout<< "new transc_equ solu "<<transcendental_solutions.first*SRSKinematic::RTD<<"   "<<transcendental_solutions.second*SRSKinematic::RTD<<std::endl;
    }
    
    return succeeded;
}

int SRSKinematicSolver::psi_picker(std::pair<double, double> act_psi, const std::string& limit, double &result)
{
    double temp_psi_first  = 0.0;
    double temp_psi_second = 0.0;
    int succeeded = 0;

    if (limit == "MIN")
    {
        if( ((act_psi.first > -SRSKinematic::PI ) && (act_psi.first < ZERO) ) && ((act_psi.second < -SRSKinematic::PI ) || (act_psi.second > ZERO)) )
        {
            result = act_psi.first;
            return succeeded;
        }
        else if ( ((act_psi.first < -SRSKinematic::PI ) || (act_psi.first > ZERO))&& ((act_psi.second < ZERO) &&(act_psi.second > -SRSKinematic::PI )) )
        {
            result = act_psi.second;
            return succeeded;
        }
        else if ((act_psi.first > ZERO) && (act_psi.second > ZERO))
        {
            temp_psi_first = act_psi.first - (2*SRSKinematic::PI );
            temp_psi_second = act_psi.second - (2*SRSKinematic::PI );
            if( ((temp_psi_first > -SRSKinematic::PI ) && (temp_psi_first < ZERO) ) && ((temp_psi_second < -SRSKinematic::PI ) || (temp_psi_second > ZERO)) )
            {
                result = temp_psi_first;
                return succeeded;
            }
            else if ( ((temp_psi_first < -SRSKinematic::PI ) || (temp_psi_first > ZERO))&& ((temp_psi_second < ZERO) &&(temp_psi_second > -SRSKinematic::PI )) )
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
        if( ((act_psi.first < SRSKinematic::PI ) && (act_psi.first > ZERO) ) && ((act_psi.second > SRSKinematic::PI ) || (act_psi.second < ZERO)) )
        {
            result = act_psi.first;
            return succeeded;
        }
        else if ( ((act_psi.first > SRSKinematic::PI ) || (act_psi.first < ZERO)) && ((act_psi.second > ZERO) && (act_psi.second < SRSKinematic::PI )) )
        {
            result = act_psi.second;
            return succeeded;
        }
        else if ((act_psi.first < ZERO) && (act_psi.second < ZERO))
        {
            temp_psi_first = act_psi.first + (2*SRSKinematic::PI );
            temp_psi_second = act_psi.second + (2*SRSKinematic::PI );
            if( ((temp_psi_first < SRSKinematic::PI ) && (temp_psi_first > ZERO) ) && ((temp_psi_second > SRSKinematic::PI ) || (temp_psi_second < ZERO)) )
            {
                result = temp_psi_first;
                return succeeded;
            }
            else if ( ((temp_psi_first > SRSKinematic::PI ) || (temp_psi_first < ZERO)) && ((temp_psi_second > ZERO) && (temp_psi_second < SRSKinematic::PI )) )
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
    //if( ((act_psi.first > -PI) && (act_psi.first < ZERO) ) &&
    //    ((act_psi.second < PI) && (act_psi.second > ZERO)) )
    if( ((act_psi.first > -SRSKinematic::PI ) && (act_psi.first < SRSKinematic::PI ) ) &&
        ((act_psi.second > -SRSKinematic::PI ) && (act_psi.second < SRSKinematic::PI )) )
    {
        return 0;
    }
    else if ( ((act_psi.first < -SRSKinematic::PI ) && (act_psi.first < ZERO) ) &&
                ((act_psi.second < SRSKinematic::PI ) && (act_psi.second > ZERO)) )
    {
        if(DEBUG)
            std::cout<< "!!! Im inside check_psi_range_bw_negPI_posPI !!!"<<std::endl;
        act_psi.first = act_psi.first + (2*SRSKinematic::PI );

        if(act_psi.first > act_psi.second)
            std::swap(act_psi.first, act_psi.second);

        return 0;
    }
    else if ( ((act_psi.first > -SRSKinematic::PI ) && (act_psi.first < ZERO) ) &&
            ((act_psi.second > SRSKinematic::PI ) && (act_psi.second > ZERO)) )
    {
        if(DEBUG)
            std::cout<< "!!! Im inside check_psi_range_bw_negPI_posPI !!!"<<std::endl;
        act_psi.second = act_psi.second - (2*SRSKinematic::PI );

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
        default                         : error_message = " This error message is not defined";
    }

    return error_message;
}




        void SRSKinematicSolver::save_tangent_joint_function(const double &an, const double &bn, const double &cn,
                                                         const double &ad, const double &bd, const double &cd,
                                                         const double &min_jtag, const double &max_jtag, const char *outputdata_file)
        {
                FILE *fp, *fp_1;  // To save data
                char jtlimit_filename[35]="../plot_data/";

                char filename[30]="../plot_data/";

                strcat (filename, outputdata_file);
                strcat (filename, ".dat");
                strcat (jtlimit_filename, outputdata_file);
                strcat (jtlimit_filename, "_jtlimit.dat");

                double theta;


                if (!(fp = fopen(filename,"w")))
                {
                        printf("Can't open file.\n");
                        exit(1);
                }
                if (!(fp_1 = fopen(jtlimit_filename,"w")))
                {
                        printf("Can't open jtlimit_filename file.\n");
                        exit(1);
                }


                for(double i = -SRSKinematic::PI ; i<= SRSKinematic::PI ; i=i+0.0174532925)
                {
                        theta = atan2( ( (an * sin(i))+(bn * cos(i))+cn ), ( (ad * sin(i))+(bd * cos(i))+cd ) );

                        //if(theta<0)
                         //       theta = theta+(2*SRSKinematic::PI );

                        fprintf(fp,"%f %f \n", (i*SRSKinematic::RTD), (theta*SRSKinematic::RTD));
                        fprintf(fp_1,"%f %f \n", (i*SRSKinematic::RTD), (min_jtag*SRSKinematic::RTD));
                        fprintf(fp_1,"%f %f \n", (i*SRSKinematic::RTD), (max_jtag*SRSKinematic::RTD));
                }

                fclose(fp);
                fclose(fp_1);

                //delete []jtlimit_filename;
                //delete []filename;

        }
        void SRSKinematicSolver::save_cosine_joint_function(const double &a, const double &b, const double &c,
                                                        const double &min_jtag, const double &max_jtag, const char* outputdata_file)
        {
                FILE *fp, *fp_1;  // To save data
                double theta;


                char jtlimit_filename[35]="../plot_data/";

                char filename[30]="../plot_data/";

                strcat (filename, outputdata_file);
                strcat (filename, ".dat");
                strcat (jtlimit_filename, outputdata_file);
                strcat (jtlimit_filename, "_jtlimit.dat");




                if (!(fp = fopen(filename,"w")))
                {
                        printf("Can't open file.\n");
                        exit(1);
                }
                if (!(fp_1 = fopen(jtlimit_filename,"w")))
                {
                        printf("Can't open jtlimit_filename file.\n");
                        exit(1);
                }               

                for(double i = -SRSKinematic::PI; i<= SRSKinematic::PI; i=i+0.0174532925)
                {
                        theta = acos((a * sin(i)) + (b * cos(i)) + c);
                        fprintf(fp,"%f %f  \n", (i*SRSKinematic::RTD), (theta*SRSKinematic::RTD));
                        fprintf(fp_1,"%f %f \n", (i*SRSKinematic::RTD), (min_jtag*SRSKinematic::RTD));
                        fprintf(fp_1,"%f %f \n", (i*SRSKinematic::RTD), (max_jtag*SRSKinematic::RTD));
                }

                fclose(fp);
                fclose(fp_1);


                //delete []jtlimit_filename;
                //delete []filename;


        }
        void SRSKinematicSolver::save_psi_file()
        {

                for(int i = 0; i< feasible_psi.size(); i++)
                {
                        if(feasible_psi.at(i).joint_number == 1)                        
                                save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, min_j1, max_j1);
                        else if(feasible_psi.at(i).joint_number == 2)
                                save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, min_j2, max_j2);
                        else if(feasible_psi.at(i).joint_number == 3)
                                save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, min_j3, max_j3);
                        else if(feasible_psi.at(i).joint_number == 5)
                                save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, min_j5, max_j5);
                        else if(feasible_psi.at(i).joint_number == 6)
                                save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, min_j6, max_j6);
                        else if(feasible_psi.at(i).joint_number == 7)
                                save_psi_file_helper((feasible_psi.at(i).joint_name).c_str(), feasible_psi.at(i).psi.at(0).first, feasible_psi.at(i).psi.at(0).second, min_j7, max_j7);
                }

                for(int i = 0; i< infeasible_psi.size(); i++)
                {
                        if(infeasible_psi.at(i).joint_number == 1)
                                save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, min_j1, max_j1);
                        else if(infeasible_psi.at(i).joint_number == 2)
                                save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, min_j2, max_j2);
                        else if(infeasible_psi.at(i).joint_number == 3)
                                save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, min_j3, max_j3);
                        else if(infeasible_psi.at(i).joint_number == 5)
                                save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, min_j5, max_j5);
                        else if(infeasible_psi.at(i).joint_number == 6)
                                save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, min_j6, max_j6);
                        else if(infeasible_psi.at(i).joint_number == 7)
                                save_psi_file_helper((infeasible_psi.at(i).joint_name).c_str(), infeasible_psi.at(i).psi.at(0).first, infeasible_psi.at(i).psi.at(0).second, min_j7, max_j7);
                }

        }

        void SRSKinematicSolver::save_psi_file_helper(const char* outputdata_file, double min, double max, double min_jt, double max_jt)
        {
                FILE *fp;
                char psi_filename[25]="../plot_data/";


                strcat (psi_filename, outputdata_file);
                strcat (psi_filename, "_psi.dat");

                if (!(fp = fopen(psi_filename,"w")))
                {
                        printf("Can't open psi_filename file.\n");
                        exit(1);
                }

                fprintf(fp,"%f %f \n", min*SRSKinematic::RTD, min_jt*SRSKinematic::RTD);
                fprintf(fp,"%f %f \n", min*SRSKinematic::RTD, max_jt*SRSKinematic::RTD);
                fprintf(fp,"%f %f \n", max*SRSKinematic::RTD, max_jt*SRSKinematic::RTD);
                fprintf(fp,"%f %f \n", max*SRSKinematic::RTD, min_jt*SRSKinematic::RTD);
                fclose(fp);

                psi_filename[0] = 0;

        }



}
