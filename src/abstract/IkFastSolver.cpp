#include <abstract/IkFastSolver.hpp>

namespace kinematics_library
{

    IkFastSolver::IkFastSolver(const std::size_t number_of_joints, const std::vector<double> jts_weight, const std::vector<std::pair<double, double> > jts_limits,
				bool (*_ComputeIkFn)(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions),
				void (*_ComputeFkFn)(const IkReal* j, IkReal* eetrans, IkReal* eerot)) :
    number_of_joints(number_of_joints), jts_weight(jts_weight), jts_limits(jts_limits), ComputeIkFn_(_ComputeIkFn), ComputeFkFn_(_ComputeFkFn)  
    {
	ComputeIkFn_ = _ComputeIkFn;
    }

    IkFastSolver::~IkFastSolver()
    {}


    bool IkFastSolver::getIK(   const std::string &base_link,
                                const base::Vector3d &target_position,
                                const base::Quaterniond &target_orientation,
                                const std::vector<double> &joint_status,
                                std::vector<double> &solution,
                                KinematicsStatus &solver_status)
    {
        IkReal eerot[9],eetrans[3];

        eetrans[0] = target_position(0);
        eetrans[1] = target_position(1);
        eetrans[2] = target_position(2);

        quaternionToRotationMatrixArray(target_orientation, eerot); 
	//ikfast::IkFastFunctions<IkReal> t;
	//t._ComputeIk = *ComputeIkFn_;

        if(ComputeIkFn_(eetrans, eerot, NULL, ik_solutions))
        {
            std::vector<std::vector<double> > optSol;

            if(pickOptimalIkSolution(joint_status, ik_solutions, optSol))
            {
                for (std::size_t i = 0; i < number_of_joints; i++)
                    solution.at(i) = optSol.at(0).at(i);

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

    bool IkFastSolver::getFK(   const std::string &base_link,
                                        const std::string &target_link,
                                        const std::vector<double> &joint_angles,
                                        base::Vector3d &fk_position,
                                        base::Quaterniond &fk_orientationZYX,
                                        KinematicsStatus &solver_status)
    {
        IkReal eerot[9],eetrans[3];
        IkReal angles[joint_angles.size()];

        for (unsigned char i=0; i < joint_angles.size(); i++)
            angles[i] = joint_angles[i];

        ComputeFkFn_(angles,eetrans,eerot);

        fk_position(0) = eetrans[0];
        fk_position(1) = eetrans[1];
        fk_position(2) = eetrans[2];

        
		// The below rotation conversion is based on ZYX. The user need to make sure that he useit properly.
		// So inorder to avoid confusion. The eigen conversion will be used.
		//rotMat2QuaternionZYX(eerot, fk_orientationZYX);

		Eigen::Matrix3d rot_mat;
		rot_mat(0,0) = eerot[0]; 		rot_mat(0,1) = eerot[1]; 		rot_mat(0,2) = eerot[2];
		rot_mat(1,0) = eerot[3]; 		rot_mat(1,1) = eerot[4]; 		rot_mat(1,2) = eerot[5];
		rot_mat(2,0) = eerot[6]; 		rot_mat(2,1) = eerot[7]; 		rot_mat(2,2) = eerot[8];

		base::Quaterniond quaternion_rot(rot_mat);
		fk_orientationZYX = quaternion_rot;

        solver_status.statuscode = KinematicsStatus::FK_FOUND;
        return true;

    }

    bool IkFastSolver::pickOptimalIkSolution(   const std::vector<double> &cur_jtang,
                                                const ikfast::IkSolutionList<IkReal> &redundantSolutions,
                                                std::vector<std::vector<double> > &optSol)
    {

        int num_solution = redundantSolutions.GetNumSolutions();
        std::vector<IkReal> sol(number_of_joints);
        std::vector<std::vector<double> > all_solution, refind_solutions;
        std::vector<int> joint_limit_exceed_solution_index;
        std::vector<double> del_sol;
        int jt_limit_exceed_sol_ct = 0;
        int ct = 0;
        std::vector<redundant_ik_solution> refined_ik_sol_container;


        all_solution.resize(num_solution, std::vector<double> (number_of_joints,0.0) );
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
            refind_solutions.resize(num_solution - jt_limit_exceed_sol_ct, std::vector<double> (number_of_joints, 0.0) );   
            // optimal solution - sorted ik solution based weighted ik sol
            optSol.resize(num_solution - jt_limit_exceed_sol_ct, std::vector<double> (number_of_joints, 0.0) );             
            // variable used to sorting the ik solution
            redundant_ik_solution refined_ik_sol[num_solution - jt_limit_exceed_sol_ct];                                    
            // variable used to sorting the ik solution
            refined_ik_sol_container.resize(num_solution - jt_limit_exceed_sol_ct);                                    

            for(int i = 0; i < num_solution; ++i)
            {
                if (joint_limit_exceed_solution_index.at(i)==0)
                {
                    for(std::size_t j = 0; j < number_of_joints; ++j)
                    {
                        del_sol.at(ct)          = del_sol.at(ct) + ( jts_weight[j] *fabs( cur_jtang[j] - all_solution[i][j] ));
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
                for(std::size_t j = 0; j < number_of_joints; ++j)
                    optSol.at(i).at(j) = refined_ik_sol_container.at(i).ik_sol.at(j);
            }

            return true;
        }
        else
            return false;

    }

    bool IkFastSolver::checkJointLimits(const ikfast::IkSolutionList<IkReal> &redundantSolutions,
                                        std::vector<std::vector<double> > &all_solution,
                                        std::vector<int> &joint_limit_exceed_solution_index,
                                        int &jt_limit_exceed_sol_ct)
    {

        int num_solution = redundantSolutions.GetNumSolutions();
        std::vector<IkReal> solvalues(number_of_joints);

        //std::cerr << "-----  Actual solution --------"<<std::endl;

        for(int i = 0; i < num_solution; ++i)
        {
            const ikfast::IkSolutionBase<IkReal>& sol = redundantSolutions.GetSolution(i);

            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0], vsolfree.size()>0?&vsolfree[0]:NULL);

            bool jt_limit_flag= false;
            for(std::size_t jn = 0; jn < number_of_joints; jn++)
            { 
                     if( !((solvalues[jn] >= jts_limits.at(jn).first) && (solvalues[jn] <= jts_limits.at(jn).second)) )
                        jt_limit_flag= true;
            }
            if(jt_limit_flag)
            {
                joint_limit_exceed_solution_index.at(i) = 1;
                jt_limit_exceed_sol_ct++;
            }

            for( std::size_t j = 0; j < number_of_joints; ++j)
            {
                all_solution[i][j] = solvalues[j];
            }
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
