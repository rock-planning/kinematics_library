#include <abstract/IkFastSolver.hpp>

namespace kinematics_library
{

IkFastSolver::IkFastSolver( const KinematicsConfig &kinematics_config, const std::vector<std::pair<double, double> > jts_limits,
                            const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, KinematicsStatus &kinematics_status):
                            jts_limits_(jts_limits)
{ 
    jts_weight_ = kinematics_config.joints_weight;
    kdl_tree_   = kdl_tree;
    kdl_chain_  = kdl_chain;

    assign_variables(kinematics_config, kdl_chain_);

    if ( !getIKFASTFunctionPtr ( kinematics_config.ikfast_lib, kinematics_status))
    {
        LOG_DEBUG("[KinematicsFactory]: Failed to retrieve IKfast function pointer.");
    }
    else
        LOG_DEBUG("[KinematicsFactory]: IKfast function pointer is retrieved sucesfully.");

}

IkFastSolver::~IkFastSolver()
{
    dlclose(ikfast_handle_);
}


bool IkFastSolver::getIKFASTFunctionPtr(const std::string ikfast_lib, KinematicsStatus &kinematics_status)
{
    char *error;

    ikfast_handle_ = dlopen ( ikfast_lib.c_str(), RTLD_LAZY );
    if ( !ikfast_handle_ )
    {
        LOG_ERROR ( "[KinematicsFactory]: Cannot open ikfast shared library. Error %s",dlerror() );
        kinematics_status.statuscode = KinematicsStatus::IKFAST_LIB_NOT_AVAILABLE;
        return false;
    }
    else        
        LOG_INFO("[KinematicsFactory]: IKfast shared lib successfully opened");

    // get the forward kinematics fuction pointer
    computeFkFn= ( void ( * ) ( const IkReal* , IkReal* , IkReal* ) )  dlsym ( ikfast_handle_, "ComputeFk" );

    if ( ( error = dlerror() ) != NULL )
    {
        LOG_ERROR ( "[KinematicsFactory]: Cannot find ComputeFk function. Error %s",dlerror() );
        dlclose ( ikfast_handle_ );
        kinematics_status.statuscode = KinematicsStatus::IKFAST_FUNCTION_NOT_FOUND;
        return false;
    }
    else
        LOG_DEBUG("[KinematicsFactory]: Found ComputeFk function in the given ikfast shared library");

    // get the inverse kinematics fuction pointer
    computeIkFn = ( bool ( * ) ( const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& ) )  dlsym ( ikfast_handle_, "ComputeIk" );
    if ( ( error = dlerror() ) != NULL )
    {
        LOG_ERROR ( "[KinematicsFactory]: Cannot find ComputeIk function. Error %s",dlerror() );
        dlclose ( ikfast_handle_ );
        kinematics_status.statuscode = KinematicsStatus::IKFAST_FUNCTION_NOT_FOUND;
        return false;
    }
    else
        LOG_DEBUG("[KinematicsFactory]: Found ComputeIk function in the given ikfast shared library");

    return true;
}

bool IkFastSolver::solveIK(const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status, std::vector<base::commands::Joints> &solution,
                           KinematicsStatus &solver_status)
{
    convertPoseBetweenDifferentFrames(kdl_tree_, target_pose, kinematic_pose_);

    getKinematicJoints(kdl_chain_, joint_status, jt_names_, current_jt_status_);

    IkReal eerot[9],eetrans[3];

    eetrans[0] = kinematic_pose_.position(0);
    eetrans[1] = kinematic_pose_.position(1);
    eetrans[2] = kinematic_pose_.position(2);

    quaternionToRotationMatrixArray(kinematic_pose_.orientation, eerot); 
    
    //ik_solution needs to be clear orelse the container holds the old value
    ik_solutions_.Clear();
    if(computeIkFn(eetrans, eerot, NULL, ik_solutions_))
    {
        std::vector<std::vector<double> > optSol;

        if(pickOptimalIkSolution(current_jt_status_, ik_solutions_, optSol))
        {
            solution.resize(optSol.size());

            for(std::vector<std::vector<double>>::iterator it = optSol.begin(); it != optSol.end(); ++it)
            {
                solution.at(it-optSol.begin()).resize(kdl_chain_.segments.size());

                for (std::size_t i = 0; i < number_of_joints_; i++)
                    solution.at(it-optSol.begin()).elements.at(i).position = it->at(i);
                solution.at(it-optSol.begin()).names = jt_names_;
            }
            solver_status.statuscode = KinematicsStatus::IK_FOUND;
            return true;
        }
        else
        {
            solver_status.statuscode = KinematicsStatus::IK_JOINTLIMITS_VIOLATED;
            return false;
        }
    }
    else
    {
        solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
        return false;
    }
}

bool IkFastSolver::solveFK(const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status)
{
    getKinematicJoints(kdl_chain_, joint_angles, jt_names_, current_jt_status_);

    IkReal eerot[9],eetrans[3];
    IkReal angles[current_jt_status_.size()];

    for (unsigned char i=0; i < current_jt_status_.size(); i++)
        angles[i] = current_jt_status_[i];

    computeFkFn(angles,eetrans,eerot);

    kinematic_pose_.position(0) = eetrans[0];
    kinematic_pose_.position(1) = eetrans[1];
    kinematic_pose_.position(2) = eetrans[2];

    // The below rotation conversion is based on ZYX. The user need to make sure that he useit properly.
    // So inorder to avoid confusion. The eigen conversion will be used.
    //rotMat2QuaternionZYX(eerot, fk_orientationZYX);

    Eigen::Matrix3d rot_mat;
    rot_mat(0,0) = eerot[0]; 		rot_mat(0,1) = eerot[1]; 		rot_mat(0,2) = eerot[2];
    rot_mat(1,0) = eerot[3]; 		rot_mat(1,1) = eerot[4]; 		rot_mat(1,2) = eerot[5];
    rot_mat(2,0) = eerot[6]; 		rot_mat(2,1) = eerot[7]; 		rot_mat(2,2) = eerot[8];

    base::Quaterniond quaternion_rot(rot_mat);
    kinematic_pose_.orientation = quaternion_rot;

    solver_status.statuscode = KinematicsStatus::FK_FOUND;

    convertPoseBetweenDifferentFrames(kdl_tree_, kinematic_pose_, fk_pose);

    return true;
}

bool IkFastSolver::pickOptimalIkSolution(   const std::vector<double> &cur_jtang, const ikfast::IkSolutionList<IkReal> &redundantSolutions,
                                            std::vector<std::vector<double> > &optSol)
{

    int num_solution = redundantSolutions.GetNumSolutions();
    std::vector<IkReal> sol(number_of_joints_);
    std::vector<std::vector<double> > all_solution, refind_solutions;
    std::vector<int> joint_limit_exceed_solution_index;
    std::vector<double> del_sol;
    int jt_limit_exceed_sol_ct = 0;
    int ct = 0;
    std::vector<redundant_ik_solution> refined_ik_sol_container;


    all_solution.resize(num_solution, std::vector<double> (number_of_joints_,0.0) );
    joint_limit_exceed_solution_index.resize(num_solution, 0);

    /*std::cerr << "-----  Current Joint angle --------"<<std::endl;
    for(int j = 0; j < 6; ++j)
    std::cerr <<cur_jtang.at(j)<<"    ";
    std::cerr<<std::endl;*/

    // check whether any solution is available which are within the joint limit and doesnt have any collision
    if (checkJointLimits(redundantSolutions, all_solution, joint_limit_exceed_solution_index, jt_limit_exceed_sol_ct))
    {
        // variables for holding the refined IK solution after checking joint limits and collision
        // variable for weighted ik sol based on min joint movement
        del_sol.resize(num_solution - jt_limit_exceed_sol_ct, 0.0);                                                    
        // variable for holding the refind ik solution
        refind_solutions.resize(num_solution - jt_limit_exceed_sol_ct, std::vector<double> (number_of_joints_, 0.0) );   
        // optimal solution - sorted ik solution based weighted ik sol
        optSol.resize(num_solution - jt_limit_exceed_sol_ct, std::vector<double> (number_of_joints_, 0.0) );             
        // variable used to sorting the ik solution
        redundant_ik_solution refined_ik_sol[num_solution - jt_limit_exceed_sol_ct];                                    
        // variable used to sorting the ik solution
        refined_ik_sol_container.resize(num_solution - jt_limit_exceed_sol_ct);                                    

        for(int i = 0; i < num_solution; ++i)
        {
            if (joint_limit_exceed_solution_index.at(i)==0)
            {
                for(std::size_t j = 0; j < number_of_joints_; ++j)
                {
                    del_sol.at(ct)          = del_sol.at(ct) + ( jts_weight_[j] *fabs( cur_jtang[j] - all_solution[i][j] ));
                    refind_solutions[ct][j]	= all_solution[i][j];     
                }     
                refined_ik_sol[ct].opt_iksol_index_value    = del_sol.at(ct);
                refined_ik_sol[ct].ik_sol                   = refind_solutions[ct];
                ct++;
            }
        }
     

        for(int i = 0; i<num_solution - jt_limit_exceed_sol_ct; i++)
            refined_ik_sol_container.at(i) = refined_ik_sol[i];

        std::sort(refined_ik_sol_container.begin(), refined_ik_sol_container.end(), compare_result());

        for(int i = 0; i<num_solution - jt_limit_exceed_sol_ct; i++)
        {
            for(std::size_t j = 0; j < number_of_joints_; ++j)
            optSol.at(i).at(j) = refined_ik_sol_container.at(i).ik_sol.at(j);
        }

        return true;
    }
    else
        return false;
}

bool IkFastSolver::checkJointLimits(const ikfast::IkSolutionList<IkReal> &redundantSolutions, std::vector<std::vector<double> > &all_solution,
                                    std::vector<int> &joint_limit_exceed_solution_index, int &jt_limit_exceed_sol_ct)
{

    int num_solution = redundantSolutions.GetNumSolutions();
    std::vector<IkReal> solvalues(number_of_joints_);

    //std::cerr << "-----  Actual solution --------"<<std::endl;

    for(int i = 0; i < num_solution; ++i)
    {
        const ikfast::IkSolutionBase<IkReal>& sol = redundantSolutions.GetSolution(i);

        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0], vsolfree.size()>0?&vsolfree[0]:NULL);

        bool jt_limit_flag= false;
        for(std::size_t jn = 0; jn < number_of_joints_; jn++)
        { 
            if( !((solvalues[jn] >= jts_limits_.at(jn).first) && (solvalues[jn] <= jts_limits_.at(jn).second)) )
            jt_limit_flag= true;
        }
        if(jt_limit_flag)
        {
            joint_limit_exceed_solution_index.at(i) = 1;
            jt_limit_exceed_sol_ct++;
        }

        for( std::size_t j = 0; j < number_of_joints_; ++j)            
            all_solution[i][j] = solvalues[j];
    }

    if(jt_limit_exceed_sol_ct == num_solution)
    {
        /*std::cerr << "-----  Joint limit exceed or Collision solution index --------"<<std::endl;
        for(int j = 0; j < num_solution; ++j)
        std::cerr <<joint_limit_exceed_solution_index.at(j)<<"    ";

        std::cerr<<std::endl;*/
        return false;
    }
    else
        return true;

}
}
