#include "kinematics_library/solver/asfour_method/IK7DoFSolver.hpp"
#include "kinematics_library/HandleKinematicConfig.hpp"

namespace kinematics_library 
{
    
    
    Ik7DoFSolver::Ik7DoFSolver ( const KinematicsConfig &kinematics_config,  const std::vector<std::pair<double, double> > &jts_limits, 
                                 const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain): jts_limits_(jts_limits)
    {
        YAML::Node input_config;
        if(!handle_kinematic_config::loadConfigFile(kinematics_config.solver_config_abs_path, kinematics_config.solver_config_filename, input_config))
            return;
        const YAML::Node& config_node = input_config["ik7dof_config"];
        ik7dof_config_ = handle_kinematic_config::getIK7DoFConfig(config_node);
        
        
        kdl_tree_       = kdl_tree;
        kdl_chain_      = kdl_chain;
        
        fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_kinematic_chain );
        
        assignVariables ( kinematics_config, kdl_chain_ );
        kdl_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
        kdl_ik_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
        
        arm_ = new kinematics_library::Arm; 
        initializeArm();
        
        
    }
    
    Ik7DoFSolver::~Ik7DoFSolver()
    {
        if ( arm_ ) 
        {
            arm_ = NULL;
            delete arm_;
        }
    }
    
    
    bool Ik7DoFSolver::solveFK ( const base::samples::Joints& joint_angles, base::samples::RigidBodyState& fk_pose, KinematicsStatus& solver_status )
    {
        getKinematicJoints(kdl_chain_, joint_angles, jt_names_, current_jt_status_);
        
        assert(current_jt_status_.size() == 7);
        for(unsigned short i = 0; i < current_jt_status_.size(); ++i)
        {
            arm_->ja_fk_in[i] = joint_angles.getElementByName(ik7dof_config_.joint_names[i]).position;
            arm_->ja_last[i]  = joint_angles.getElementByName(ik7dof_config_.joint_names[i]).position;
        }
        
        int success = fkArm();
        if(success ==1)
        {
            kinematic_pose_ = getRBSPose(arm_->T_Base_2_TCP_fk_out);
            
            convertPoseBetweenDifferentFrames(kdl_tree_, kinematic_pose_, fk_pose);
            solver_status.statuscode = KinematicsStatus::FK_FOUND;   
            return true;
        }
        else 
        {
            solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
            return false;
        }
    }
    
    bool Ik7DoFSolver::solveIK ( const base::samples::RigidBodyState target_pose, const base::samples::Joints& joint_status, std::vector< base::commands::Joints >& solution, KinematicsStatus& solver_status )
    {
        convertPoseBetweenDifferentFrames(kdl_tree_, target_pose, kinematic_pose_);
        getKinematicJoints(kdl_chain_, joint_status, jt_names_, current_jt_status_);
        
        //for(auto name: jt_names_)
        //    std::cout<<name<<std::endl;
        
        std::vector<double> joints_actual;
        //std::cout<<"b4 getKinematicJoints"<<std::endl;
        //std::cout<<ik7dof_config_.joint_names.size()<<std::endl;
        
        //getKinematicJoints(kdl_chain_, joint_status, ik7dof_config_.joint_names, joints_actual);
        
        //std::cout<<"b4 assert"<<std::endl;
        
        //assert(joints_actual.size() == 7);
        //std::cout<<"assert"<<std::endl;
        
        for(unsigned short i = 0; i < ik7dof_config_.joint_names.size(); ++i)
        {
            arm_->ja_last[i]  = joint_status.getElementByName(ik7dof_config_.joint_names[i]).position;
        }
        
        setDesiredPose(kinematic_pose_, arm_->pos_ik_in, arm_->Rbase2tcp);
        
        rotationMatrix2zyxEuler(arm_->Rbase2tcp, arm_->rot_ik_in);
        
        int success = ikArm();
        if(success ==1)
        {
            base::samples::Joints joints_least_effort = listJointsInDesiredOrder(arm_->ja_ik_out, jt_names_, ik7dof_config_.joint_names);
            if(validateJointLimits(joints_least_effort,  jts_limits_ ) && !joints_least_effort.names.empty())
            {
                solution.resize(1);
                joints_least_effort.time = target_pose.time;
                solution[0] = joints_least_effort;
                solver_status.statuscode = KinematicsStatus::IK_FOUND;
                return true;
            }
            else
            {
                std::vector<base::samples::Joints> valid_solution_list;
                for(unsigned short i = 0; i < 8; ++i)
                {
                    base::samples::Joints joint_samples = listJointsInDesiredOrder(arm_->ja_all[i], jt_names_, ik7dof_config_.joint_names);
                    if(validateJointLimits(joint_samples, jts_limits_))
                    {
                        valid_solution_list.push_back(joint_samples);
                    }
                }
                if(valid_solution_list.size()>0)
                {
                    //std::cout<<"valid_solution_list"<<std::endl;
                    solution = pickOptimalSolution(valid_solution_list, joint_status);
                    solver_status.statuscode = KinematicsStatus::IK_FOUND;
                    return true;
                }
                else
                {
                    //std::cout<<"invalid_solution_list"<<std::endl;
                    solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
                    return false;
                }
            }      
        }
        else
        {
            solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
            return false;
        } 
    }
    
    int Ik7DoFSolver::initializeArm()
    {
        double bs = ik7dof_config_.offset_base_shoulder;
        double se = ik7dof_config_.offset_shoulder_elbow;
        double ew = ik7dof_config_.offset_elbow_wrist;
        double wt = ik7dof_config_.offset_wrist_tool;
        
        double dh_d[8]  = {   bs,  0.0,   se,  0.0,   ew,  0.0,  0.0,  0.0 };
        double dh_a[8]  = {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   wt,  0.0 };
        double dh_ca[8] = {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0 };
        double dh_sa[8] = { -1.0,  1.0, -1.0,  1.0, -1.0, -1.0,  0.0, -1.0 };
        double dh_do[8] = {-PI/2,  PI/2,  0.0,  0.0, PI/2,-PI/2,  0.0,-PI/2 };
        
        unsigned short i;
                
        for(i = 0; i < 8; i++)
        {
            arm_->dh_d[i] = dh_d[i];
            arm_->dh_a[i] = dh_a[i];
            arm_->dh_ca[i] = dh_ca[i];
            arm_->dh_sa[i] = dh_sa[i];
            //arm_->dh_do[i] = dh_do[i];
            arm_->dh_do[i] = ik7dof_config_.theta_offsets[i];
            //std::cout<<arm_->dh_do[i]<<std::endl;

        }
        // define the angle limits
        double j_max[7] = { PI, PI, PI, PI, PI, PI, PI };
        double j_min[7] = {-PI,-PI,-PI,-PI,-PI,-PI,-PI };
        
        for(i = 0; i < 7; i++){
            arm_->j_max[i] = j_max[i];
            arm_->j_min[i] = j_min[i];
        }
        arm_->mode = MANIPULATION;
        arm_->ze_mode = ik7dof_config_.ze_mode;
        
        return 1;
    }
    
    int Ik7DoFSolver::fkArm()
    {
        unsigned short i;
        double dh_t;
        for(i = 0; i < 8; i++){
            
            //Adaption to the joint delta angle offsets
            if(i == 7){
                dh_t = arm_->dh_do[i];
            }
            else{
                
                dh_t = arm_->ja_fk_in[i] + arm_->dh_do[i];
            }
            
            // calc sin and cos just once
            double st = sin(dh_t);
            double ct = cos(dh_t);
            double sa = arm_->dh_sa[i];
            double ca = arm_->dh_ca[i];
            
            // build the transformation matrix from frame to frame
            double T_Fr_2_Fr[16];
            T_Fr_2_Fr[0] =  ct;    T_Fr_2_Fr[4] = -st*ca;   T_Fr_2_Fr[8]  =  st*sa;   T_Fr_2_Fr[12] = ct*arm_->dh_a[i];
            T_Fr_2_Fr[1] =  st;    T_Fr_2_Fr[5] =  ct*ca;   T_Fr_2_Fr[9]  = -ct*sa;   T_Fr_2_Fr[13] = st*arm_->dh_a[i];
            T_Fr_2_Fr[2] = 0.0;    T_Fr_2_Fr[6] =     sa;   T_Fr_2_Fr[10] =     ca;   T_Fr_2_Fr[14] =    arm_->dh_d[i];
            T_Fr_2_Fr[3] = 0.0;    T_Fr_2_Fr[7] =    0.0;   T_Fr_2_Fr[11] =    0.0;   T_Fr_2_Fr[15] =             1.0;
            
            /*for(int j = 0; j<16; ++j)
            {
                arm_->T(i, j) = T_Fr_2_Fr[j]; 
            }*/
            
            
            if(i == 0){
                copyMat4x4(T_Fr_2_Fr, arm_->T_Base_2_TCP_fk_out);
            }
            else
            {
                double T_Base_2_TCP_tmp[16];
                copyMat4x4(arm_->T_Base_2_TCP_fk_out, T_Base_2_TCP_tmp);
                multMatMat(T_Base_2_TCP_tmp, T_Fr_2_Fr, arm_->T_Base_2_TCP_fk_out);
            }
            /*if(i == 3 || i == 7)
            {
                std::string link = (i==3)? "elbow":"tcp";
                LOG_DEBUG_S<<"Transform from  base to "<<link<<std::endl;
                //print_as_rbs(arm_->T_Base_2_TCP_fk_out);
                
                //print_as_matrix(arm_->T_Base_2_TCP_fk_out);
            }*/
        }
        return 1;
    }
    
    int Ik7DoFSolver::ikArm()
    {
        double w_t[3];
        
        double ls = arm_->dh_d[0];
        double lu = arm_->dh_d[2];
        double lf = arm_->dh_d[4];
        double lh = arm_->dh_a[6];
        
        // input arm->rot_ination in 3x3 matrix
        double Rbase2tcp[9];
        double n[3], vec[3], w[3];
        //Eul2RotMat(arm->rot_ik_in, Rbase2tcp);
        //print_ja(Rbase2tcp, 9);
        
        //ik_7dof::copy_array(arm_->Rbase2tcp, Rbase2tcp, 9);
        //print_as_matrix(Rbase2tcp, 3,3);
        
        if(arm_->mode == MANIPULATION){
            // get the arm->pos_inition of the wrist intersection point
            n[0] = arm_->Rbase2tcp[6];
            n[1] = arm_->Rbase2tcp[7];
            n[2] = arm_->Rbase2tcp[8];
            
            scaleVec(n, -lh, vec);
            addVec(arm_->pos_ik_in, vec, w);
            
            // subtract the shoulder offset to facilitate the equations
            w_t[0] = w[0]; w_t[1] = w[1]; w_t[2] = w[2] - ls;
            
        }else{   // arm->mode= locomotion
            w[0] = arm_->pos_ik_in[0]; w[1] = arm_->pos_ik_in[1]; w[2] = arm_->pos_ik_in[2];
            w_t[0] = arm_->pos_ik_in[0]; w_t[1] = arm_->pos_ik_in[1]; w_t[2] = arm_->pos_ik_in[2] - ls;
        }
        
        // quit if the position is not reachable
        double d = normVec(w_t);
        if(lu + lf - d < -kinematics_library::EPSILON){
            printf("[Arm Inverse Kinematics] The desired position is out of reach!\n");
            return 0;
        }
        
        // if the elbow is straight, some equations simplify and some problems have to be solved differently
        unsigned short elbowIsStraight;
        if(fabs(lu + lf - d) < kinematics_library::EPSILON){
            elbowIsStraight = 1;
            LOG_DEBUG_S<<"Straight"<<std::endl;
        }else{
            elbowIsStraight = 0;
        }
        
        // get the ze_min and ze_max (by solving sqrt(xw^2+yw^2) <= r1+r2)
        double e1[3], e2[3];
        double ze_min, ze_max;
        if(elbowIsStraight){
            // if we have a straight elbow, the elbow arm_->pos_ine is simple to determine
            double ratio = lu / (lu + lf);
            scaleVec(w_t, ratio, e1);
            e1[2] += ls;
            
            // both elbow solutions are equal
            e2[0] = e1[0]; e2[1] = e1[1]; e2[2] = e1[2];
            
            // update the z plane parameters for higher levels
            arm_->ze_out = arm_->ze_max = arm_->ze_min = e1[2];
            
        }else{
            // if the elbow is not straight, it is way more complex
            
            // first the limits for ze have to be calculated
            double sqrt_term = sqrt((w_t[0]*w_t[0] + w_t[1]*w_t[1]) * (lf*lf * (-(lf*lf) + 2.0*lu*lu + 2.0*d*d)
            + lu*lu*(-lu*lu+2.0*d*d)  + w_t[0]*w_t[0]*(-w_t[0]*w_t[0] - 2.0*w_t[1]*w_t[1] - 2.0*w_t[2]*w_t[2])
            + w_t[1]*w_t[1] * (-w_t[1]*w_t[1] - 2*w_t[2]*w_t[2]) - w_t[2]*w_t[2]*w_t[2]*w_t[2]));
            
            ze_max = (w_t[2]*(lu*lu - lf*lf + d*d) + sqrt_term) / (2.0*d*d);
            ze_min = (w_t[2]*(lu*lu - lf*lf + d*d) - sqrt_term) / (2.0*d*d);
            
            // transform the limits in the base coordinate system and update the z plane parameters for higher levels
            arm_->ze_max = ze_max;
            arm_->ze_min = ze_min;
            
            // definition of the ze value
            double ze;
            switch(arm_->ze_mode)
            {    
                case MANUAL:
                {
                // in manual arm->modetransform ze in the shoulder coordinates system and limit the inputs if necessary
                    ze = arm_->ze_in - ls;
                    if(ze > arm_->ze_max)
                        ze = arm_->ze_max;
                    else if(ze < arm_->ze_min)
                        ze = arm_->ze_max;
                
                    arm_->ze_out = ze;
                    break;
                }
                case AUTO_YMAX:
                {
                // a simple working solution would be
                // ze = (ze_max+ze_min)/2;
                // but in automatic arm_->modewe want to calculate the z plane which maximizes the
                // y-coordinate of the elbow
                
                // specify some help variables
                    double d_xz_sq = w_t[0]*w_t[0] + w_t[2]*w_t[2];
                    double K = lu*lu - lf*lf + d*d;
                
                    double term1 = K * w_t[2] * d_xz_sq;
                    double term2 = w_t[1] * w_t[2] * sqrt( d_xz_sq * (-K*K + 4.0 * lu*lu * d*d) );
                    double term3 = 2.0 * d_xz_sq * d*d;
                
                    double ze_ymax1 = (term1 - term2) / term3;
                //double ze_ymax2 = (term1 + term2) / term3;    // this solution always seems to be wrong
                
                    ze = ze_ymax1;
                // transform the z coordinates in the base coordinate system (for higher levels)
                    arm_->ze_out = ze + ls;
                    break;
                
                }
                case AUTO_ZSTABLE:
                {    
                    arm_->ze_out = (arm_->ze_max+arm_->ze_min)/2;
                    break;
                }
                case AUTO_ZMAX:
                {    
                    arm_->ze_out = floor(arm_->ze_max * 1000.0) / 1000.0;
                    break;
                }
                default:
                    arm_->ze_out = floor(arm_->ze_max * 1000.0) / 1000.0;
            }
            ze = arm_->ze_out;
            // squared radii around shoulder and wrist at z-plane
            double r1_sq = lu*lu - ze*ze;                            // with M1 = (0; 0; ze)
            double r2_sq = lf*lf - (ze - w_t[2]) * (ze - w_t[2]);    // with M2 = (w[0]; w[1]; w[2]-ze)
            
            // solve for two arm_->pos_insible arm_->pos_initions for the elbow (two solution per z-plane)
            double d_xy_sq = w_t[0]*w_t[0] + w_t[1]*w_t[1];
            double C = (r1_sq - r2_sq + d_xy_sq) / 2.0;
            double minus_p_half = C * w_t[1] / d_xy_sq;
            
            e1[1] =  minus_p_half + sqrt( minus_p_half*minus_p_half - (C*C - r1_sq * w_t[0]*w_t[0]) / d_xy_sq);
            e2[1] =  minus_p_half - sqrt( minus_p_half*minus_p_half - (C*C - r1_sq * w_t[0]*w_t[0]) / d_xy_sq);
            
            e1[2] = ze + ls;
            e2[2] = e1[2];
            
            if(fabs(w_t[0]) < kinematics_library::EPSILON){
                // when the x-component is zero, the normal equations simplify or cannot be used
                e1[1] =  minus_p_half;                     // because the sqrt term is zero
                e2[1] =  minus_p_half;                     // because the sqrt term is zero
                
                // use the Euclidean distance instead to avoid division by 0
                double xe_sq = lu*lu - e1[1]*e1[1] - ze*ze;
                if(xe_sq > kinematics_library::EPSILON){
                    e1[0] = sqrt(lu*lu - e1[1]*e1[1] - ze*ze); // use the Euclidean distance instead
                    e2[0] =-sqrt(lu*lu - e2[1]*e2[1] - ze*ze); // to avoid division by 0
                }else{
                    e1[0] = e2[0] = 0.0;
                }
                
            }else{
                // the usual case
                e1[1] =  minus_p_half + sqrt( minus_p_half*minus_p_half - (C*C - r1_sq * w_t[0]*w_t[0]) / d_xy_sq);
                e2[1] =  minus_p_half - sqrt( minus_p_half*minus_p_half - (C*C - r1_sq * w_t[0]*w_t[0]) / d_xy_sq);
                
                e1[0] =  (C - w_t[1] * e1[1]) / w_t[0];
                e2[0] =  (C - w_t[1] * e2[1]) / w_t[0];
            }
        }
        
        
        // determine the vectors of the axis of the coordinate systems at joint4
        // from now on, two solutions arm_->pos_insible (elbow up or down)
        double e[3];
        unsigned short i;
        for(i = 0; i < 2; i++){
            if(i == 0){
                e[0] = e1[0];   e[1] = e1[1];   e[2] = e1[2];
            }else{
                e[0] = e2[0];   e[1] = e2[1];   e[2] = e2[2];
            }
            
            // calculate the vectors of the elbow coordinate system
            double e_minus[3], rf[3], ru[3];
            double rs[3] = {0.0, 0.0, ls};
            
            scaleVec(e, -1, e_minus);
            addVec(w, e_minus, rf);
            addVec(rs, e_minus, ru);
            
            double x4_v[3], y4_v[3], z4_v[3];
            double z4_v_cross_ru[3];
            double divisor = normVec(rf);
            scaleVec(rf, 1/divisor, z4_v);
            crossVec(z4_v, ru, z4_v_cross_ru);
            scaleVec(z4_v_cross_ru, 1/normVec(z4_v_cross_ru), y4_v);
            crossVec(y4_v, z4_v, x4_v);
            
            double T_elbow[16];
            T_elbow[0] = x4_v[0];   T_elbow[4] = y4_v[0];   T_elbow[8]  = z4_v[0];   T_elbow[12] = e[0];
            T_elbow[1] = x4_v[1];   T_elbow[5] = y4_v[1];   T_elbow[9]  = z4_v[1];   T_elbow[13] = e[1];
            T_elbow[2] = x4_v[2];   T_elbow[6] = y4_v[2];   T_elbow[10] = z4_v[2];   T_elbow[14] = e[2];
            T_elbow[3] = 0.0;       T_elbow[7] = 0.0;       T_elbow[11] = 0.0;       T_elbow[15] = 1.0;
            
            //    cout << "[arm_kinematics] elbow pos determined" << endl;
            //    unsigned short col, row;
            //    for(row = 0; row < 4; row++){
            //      for(col = 0; col < 4; col++){
            //        cout << T_elbow[row + 4*col] << " ";
            //      }
            //      cout << endl;
            //    }
            
            // calculate the joint angles
            double theta[7];
            unsigned short j;
            for(j = 0; j < 2; j++){
                if(j == 0){
                    theta[0] = atan2(T_elbow[13], T_elbow[12]);       // normal shoulder
                }else{
                    theta[0] = atan2(-T_elbow[13], -T_elbow[12]);     // inverse shoulder
                }
                double c1 = cos(theta[0]);
                double s1 = sin(theta[0]);
                
                theta[1] = atan2(c1*T_elbow[12] + s1*T_elbow[13], T_elbow[14]-ls);
                double c2 = cos(theta[1]);
                double s2 = sin(theta[1]);
                
                // if straight arm, theta 3 equal to last theta3, because it can be
                // defined by oneself
                if(elbowIsStraight){
                    theta[2] = arm_->ja_last[2];
                }else{
                    theta[2] = atan2(-s1*T_elbow[8] + c1*T_elbow[9], c1*c2*T_elbow[8] + s1*c2*T_elbow[9] - s2*T_elbow[10]);
                }
                
                double T_04[16];
                // theta 4 is zero for sure if the arm is straight
                if(elbowIsStraight){
                    theta[3] = 0.0;
                    
                    // but in this case the orientation of the elbow is different to
                    // the input -> so it has to be calculated with the FK
                    double T_Base_2_TCP[16];
                    unsigned short q;
                    for(q = 0; q < 4; q++){
                        double st = sin(theta[q]);
                        double ct = cos(theta[q]);
                        double sa = arm_->dh_sa[q];
                        double ca = arm_->dh_ca[q];
                        
                        // build the transformation matrix from frame to frame
                        double T_Fr_2_Fr[16];
                        T_Fr_2_Fr[0] =  ct;    T_Fr_2_Fr[4] = -st*ca;   T_Fr_2_Fr[8]  =  st*sa;   T_Fr_2_Fr[12] = ct*arm_->dh_a[q];
                        T_Fr_2_Fr[1] =  st;    T_Fr_2_Fr[5] =  ct*ca;   T_Fr_2_Fr[9]  = -ct*sa;   T_Fr_2_Fr[13] = st*arm_->dh_a[q];
                        T_Fr_2_Fr[2] = 0.0;    T_Fr_2_Fr[6] =     sa;   T_Fr_2_Fr[10] =     ca;   T_Fr_2_Fr[14] =    arm_->dh_d[q];
                        T_Fr_2_Fr[3] = 0.0;    T_Fr_2_Fr[7] =    0.0;   T_Fr_2_Fr[11] =    0.0;   T_Fr_2_Fr[15] =             1.0;
                        
                        // multiply iteratively the transformation matrix from frame to frame with the transformation matrix to the base
                        if(q == 0){
                            copyMat4x4(T_Fr_2_Fr, T_Base_2_TCP);
                        }else{
                            double T_Base_2_TCP_tmp[16];
                            copyMat4x4(T_Base_2_TCP, T_Base_2_TCP_tmp);
                            multMatMat(T_Base_2_TCP_tmp, T_Fr_2_Fr, T_Base_2_TCP);
                        }
                        
                        copyMat4x4(T_Base_2_TCP, T_04);
                    }
                    
                }else{
                    // if the arm is bended, the elbow angle has to be calculated
                    theta[3] = atan2(-(c1*s2*T_elbow[0] + s1*s2*T_elbow[1] + c2*T_elbow[2]), c1*s2*T_elbow[8] + s1*s2*T_elbow[9] + c2*T_elbow[10]);
                    //std::cout<<std::endl<<"theta3: "<<theta[3]<<"  ";
                    copyMat4x4(T_elbow, T_04);
                }
                
                // calculate the transformation from elbow to tcp based on the first four
                // angles
                double ct8 = cos(arm_->dh_do[7]); double ca8 = arm_->dh_ca[7];
                double st8 = sin(arm_->dh_do[7]); double sa8 = arm_->dh_sa[7];
                double T_78[16];
                double T_78_inv[16];
                double T_08[16];
                double T_07[16];
                double T_04_inv[16];
                double T_47[16];
                
                T_78[0] = ct8; T_78[4] =-st8*ca8;  T_78[8]  = st8*sa8; T_78[12] = 0.0;
                T_78[1] = st8; T_78[5] = ct8*ca8;  T_78[9]  =-ct8*sa8; T_78[13] = 0.0;
                T_78[2] = 0.0; T_78[6] = sa8;      T_78[10] = ca8;     T_78[14] = 0.0;
                T_78[3] = 0.0; T_78[7] = 0.0;      T_78[11] = 0.0;     T_78[15] = 1.0;
                
                
                invertMat(T_78, T_78_inv);
                //std::cout<<"\n---------------------------T_78inv------------------------"<<std::endl;
                //print_as_matrix(T_78_inv, 4,4);
                buildTransformationMat(arm_->Rbase2tcp, arm_->pos_ik_in, T_08);
                //std::cout<<"---------------------------T_08------------------------"<<std::endl;
                //print_as_matrix(T_08, 4,4);
                multMatMat(T_08, T_78_inv, T_07);
                //std::cout<<"---------------------------T_07------------------------"<<std::endl;
                //print_as_matrix(T_07, 4,4);
                
                invertMat(T_04, T_04_inv);
                
                //std::cout<<"---------------------------T_04inv------------------------"<<std::endl;
                //print_as_matrix(T_04_inv, 4,4);
                multMatMat(T_04_inv, T_07, T_47);
                
                //std::cout<<"---------------------------T_47------------------------"<<std::endl;
                //print_as_matrix(T_47, 4,4);
                //std::cout<<"---------------------------T_47------------------------"<<std::endl;
                unsigned short k;
                for(k = 0; k < 2; k++){
                  
                        /*theta[5] = acos(-T_47[10]);
                        //theta[5] = atan2(-sqrt(T_47[8]*T_47[8]+ T_47[9]*T_47[9]), -T_47[10]);
                        
                        if(k==0)
                            theta[5] = atan2(sqrt(T_47[8]*T_47[8]+ T_47[9]*T_47[9]), -T_47[10]);
                        else
                            theta[5] = atan2(-sqrt(T_47[8]*T_47[8]+ T_47[9]*T_47[9]), -T_47[10]);
                        
                        //std::cout<<"theta5: "<<theta[5]<<"  ";
                        double s6 = sin(theta[5]);
                        if(fabs(sin(theta[5]))> ZERO_PRECISION)
                        {
                            if(k == 0)
                            {
                                theta[4] = atan2(-T_47[9]/s6, -T_47[8]/s6);    // normal wrist
                                theta[6] = atan2(T_47[6]/s6, -T_47[2]/s6);
                            }
                            else
                            {
                                theta[4] = atan2(-T_47[9]/s6, T_47[8]/s6);    // normal wrist
                                theta[6] = atan2(-T_47[6]/s6, -T_47[2]/s6);
                                //theta[4] = atan2(T_47[9], T_47[8]);      // inverse wrist
                                //theta[6] = atan2(-T_47[6], T_47[2]);
                            }       
                        }
                        else
                        {
                            theta[4] = arm_->ja_last[4] + arm_->dh_do[4];
                            double c5 = cos(theta[4]);
                            double s5 = sin(theta[4]);
                            theta[6] = atan2( -c5*T_47[4]-s5*T_47[5], s5*T_47[0]+c5*T_47[1]);
                        }
                        //std::cout<<"theta4: "<<theta[4]<<"  ";
                        //      std::cout<<"theta6: "<<theta[6]<<"---------";
                        //theta[5] = atan2(-(T_47[8]*c5 + T_47[9]*s5), -T_47[10]);
                        
                        //theta[6] = atan2(-(T_47[1]*c5 - T_47[0]*s5), -(T_47[5]*c5 - T_47[4]*s5));*/
                        
                        if(k == 0){
                            theta[4] = atan2(-T_47[9], -T_47[8]);    // normal wrist
                        }else{
                            theta[4] = atan2(T_47[9], T_47[8]);      // inverse wrist
                        }
                        double c5 = cos(theta[4]);
                        double s5 = sin(theta[4]);
                        
                        theta[5] = atan2(-(T_47[8]*c5 + T_47[9]*s5), -T_47[10]);
                        
                        theta[6] = atan2(-(T_47[1]*c5 - T_47[0]*s5), -(T_47[5]*c5 - T_47[4]*s5));    
                        
                    unsigned short solution_nr = i*4 + j*2 + k;
                    unsigned short dof;
                    for(dof=0; dof<7; dof++){
                        // transfer the joint angles to the robot
                        double ja = theta[dof] - arm_->dh_do[dof];
                        
                        // and place them in an interval of -pi ... pi
                        arm_->ja_all[solution_nr][dof] = doubleModulo(ja + PI, 2*PI) - PI;
                        
                    }
                } // end inverse wrist
            }   // end inverse shoulder
        }     // end elbow up/down
        // find the best solution
        unsigned short best_solution = 1;
        double min_effort = 10000;
        
        unsigned short sol_nr;
        for(sol_nr = 0; sol_nr < 8; sol_nr++){
            double effort = 0.0;
            unsigned short dof_nr;
            for(dof_nr = 0; dof_nr < 7; dof_nr++){
                effort = effort + fabs(arm_->ja_all[sol_nr][dof_nr] - arm_->ja_last[dof_nr]);
            }
            if(effort < min_effort){
                min_effort = effort;
                best_solution = sol_nr;
            }
        }
        
        unsigned short dof_nr;
        for(dof_nr = 0; dof_nr < 7; dof_nr++){
            arm_->ja_ik_out[dof_nr] = arm_->ja_all[best_solution][dof_nr];
        }
        
        return 1;
    }


    base::samples::RigidBodyState Ik7DoFSolver::getRBSPose(double* homogeneous_mat)
    {
        base::samples::RigidBodyState fk_pose;
        fk_pose.position(0) = homogeneous_mat[12];
        fk_pose.position(1) = homogeneous_mat[13];
        fk_pose.position(2) = homogeneous_mat[14];
        
        Eigen::Matrix3d rot_mat;
        rot_mat(0,0) = homogeneous_mat[0];   rot_mat(0,1) = homogeneous_mat[4];  rot_mat(0,2) = homogeneous_mat[8];
        rot_mat(1,0) = homogeneous_mat[1];   rot_mat(1,1) = homogeneous_mat[5];  rot_mat(1,2) = homogeneous_mat[9];
        rot_mat(2,0) = homogeneous_mat[2];   rot_mat(2,1) = homogeneous_mat[6];  rot_mat(2,2) = homogeneous_mat[10];
                
        base::Quaterniond quaternion_rot(rot_mat);
        fk_pose.orientation = quaternion_rot; 
        
        LOG_INFO_S<<"x: "<<fk_pose.position(0)<<" y: "<<fk_pose.position(1)<<" z: "<<fk_pose.position(2)<<std::endl;
        LOG_INFO_S<<"qx: "<<fk_pose.orientation.x()<<" qy: "<<fk_pose.orientation.y()<<" qz: "<<fk_pose.orientation.z()<<" qw: "<<fk_pose.orientation.w()<<std::endl;
        
        fk_pose.sourceFrame = kinematic_pose_.sourceFrame;
        fk_pose.targetFrame = kinematic_pose_.targetFrame;
        
        return fk_pose;
    }

    void Ik7DoFSolver::setDesiredPose(const base::samples::RigidBodyState& rbs_pose, double* pos_vec, double* rot_mat)
    {
        pos_vec[0] = rbs_pose.position(0);
        pos_vec[1] = rbs_pose.position(1);
        pos_vec[2] = rbs_pose.position(2);
        base::MatrixXd rot_eigen = rbs_pose.orientation.toRotationMatrix();
        eigenMat3x3ToArray(rot_eigen, rot_mat);      
    }
    
    base::samples::Joints Ik7DoFSolver::listJointsInDesiredOrder(double* joint_values, const std::vector< std::__cxx11::string >& desired_joint_names_order, const std::vector< std::__cxx11::string >& actual_joint_names_order)
    {   
        assert(desired_joint_names_order.size() == actual_joint_names_order);
       /* for(unsigned short i = 0; i < actual_joint_names_order.size(); ++i)
        {
            std::cout<<actual_joint_names_order[i]<<"*********"<<desired_joint_names_order[i]<<": "<<joint_values[i]<<std::endl;
        }*/
        std::vector<double> joint_vec;        
        base::samples::Joints joint_samples;
        joint_samples.resize(desired_joint_names_order.size());
        for(unsigned short i = 0; i < desired_joint_names_order.size(); ++i)
        {
            if(desired_joint_names_order[i] == actual_joint_names_order[i])
            {
                //joint_vec.push_back(joint_values[i]);
                joint_samples.names[i] = desired_joint_names_order[i];
                joint_samples.elements[i].position = joint_values[i];
            }
            else
            {
                auto it = std::find(actual_joint_names_order.begin(), actual_joint_names_order.end(), desired_joint_names_order[i]);
                if(it != actual_joint_names_order.end())
                {
                    unsigned index = std::distance(actual_joint_names_order.begin(), it);
                    //joint_vec.push_back(joint_values[index]);
                    joint_samples.names[i] = desired_joint_names_order[i];
                    joint_samples.elements[i].position = joint_values[index];
                }
                else
                {
                    LOG_FATAL_S<<"[kinematics_library::IK7DoFSolver] Cannot find the urdf specified joint name in the ik joint name list";
                }
            }
        }
        //base::samples::Joints joint_samples;
        //joint_samples.Positions(joint_vec, desired_joint_names_order);
        
       /* for(unsigned short i = 0; i < joint_samples.names.size(); ++i)
        {
            std::cout<<joint_samples.names[i]<<joint_samples.elements[i].position<<std::endl;
        }*/
        
        return joint_samples;
        
    }
    
    bool Ik7DoFSolver::validateJointLimits(const base::samples::Joints& joint_values, const std::vector< std::pair< double, double > >& jts_limits)
    {
        assert(joint_values.names.size() == jts_limits.size());
        
        for(unsigned short i = 0; i < joint_values.elements.size(); ++i)
        {
            if(joint_values.elements[i].position < jts_limits[i].first || joint_values.elements[i].position > jts_limits[i].second)
                return false;
        }
        return true;
    }

    std::vector<base::samples::Joints> Ik7DoFSolver::pickOptimalSolution(const std::vector< base::samples::Joints >& solution_list, const base::samples::Joints& joints_actual)
    {
        std::vector<SolutionWithScore> sol_list_scored;
        for(unsigned short i = 0; i < solution_list.size(); ++i)
        {
            double sum = 0;
            base::samples::Joints sol =  solution_list[i];
            for(unsigned short j = 0; j< sol.elements.size(); ++j)
            {
                sum = sum + fabs(sol.elements[i].position - joints_actual.getElementByName(sol.names[i]).position);
            }
            SolutionWithScore sol_scored(sol, sum);
            sol_list_scored.push_back(sol_scored);
        }
        std::sort(sol_list_scored.begin(), sol_list_scored.end(), CompareSolutions());
        
        std::vector<base::samples::Joints> ordered_sol_list;
        
        for(auto sol: sol_list_scored)
            ordered_sol_list.push_back(sol.joints_value);
        
        return ordered_sol_list;
    }
}
