#include "kinematics_library/HandleKinematicConfig.hpp"

namespace kinematics_library
{

double objectives(const std::vector<double>& x, std::vector<double>& grad, void* data)
{   
    OptSolver *c = (OptSolver *) data;
    return c->problem_formulation_.getOverallCost(x, grad);
}

//double jointsLimitsConstraint(const std::vector<double>& x, std::vector<double>& grad, void* data)
void jointsLimitsConstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
{
    // m = number of constraints
    // n = size of x   
    
    OptSolver *c = (OptSolver *) data;
    
    c->problem_formulation_.getJointsLimitsConstraintCost( m, n, x, result, grad); 
    //std::cout<<"size "<<x.size()<<"  "<<grad.size()<<std::endl;
}

OptSolver::OptSolver ( const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree, const KDL::Chain &kdl_chain, const KDL::Chain &kdl_kinematic_chain)
{
    kdl_tree_            = kdl_tree;
    kdl_chain_           = kdl_chain;
    kdl_kinematic_chain_ = kdl_kinematic_chain;
    jts_limits_          = jts_limits;

}

OptSolver::~OptSolver()
{
    if ( fk_kdlsolver_pos_ ) 
    {
        delete fk_kdlsolver_pos_;
        fk_kdlsolver_pos_ = nullptr;
    }
}

bool OptSolver::loadKinematicConfig( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status)
{
    // assign the config
    YAML::Node input_config;
    // check whether the config could be loaded or not.
    if(!handle_kinematic_config::loadConfigFile(kinematics_config.solver_config_abs_path, kinematics_config.solver_config_filename, input_config))
    {
        LOG_ERROR("[OptSolver]: Unable to load kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(), 
                    kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::NO_CONFIG_FILE;
        return false;
    }
     
    const YAML::Node& opt_ik_config_node = input_config["opt_ik_config"];
    
    if(!handle_kinematic_config::getOptIKConfig(opt_ik_config_node, opt_param_))
    {
        LOG_ERROR("[OptSolver]: Unable to read kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(), 
                    kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = KinematicsStatus::CONFIG_READ_ERROR;
        return false;
    }
	

    fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_kinematic_chain_ );

    assignVariables ( kinematics_config, kdl_chain_ );
    kdl_jt_array_.resize ( kdl_chain_.getNrOfJoints() );
    
    // initialise the problem
    if(!initialiseProblem(opt_param_))
        return false;

    kinematics_status.statuscode = KinematicsStatus::SUCCESS;
    return true;
}

bool OptSolver::solveIK (const base::samples::RigidBodyState &target_pose, 
                            const base::samples::Joints &joint_status,
                            std::vector<base::commands::Joints> &solution,
                            KinematicsStatus &solver_status )
{
    // get the proper frame for the IK solver
    if(!convertPoseBetweenDifferentFrames(kdl_tree_, joint_status, target_pose, kinematic_pose_))
    {
        solver_status.statuscode = KinematicsStatus::KDL_CHAIN_FAILED;
        return false;
    }

    // get the joint status only for the kinematic chain that this solver intended to solve.
    getKinematicJoints ( kdl_chain_, joint_status, jt_names_, current_jt_status_ );
    // assign the current joint angles as the optimization variables
    for(std::size_t i = 0; i < number_of_joints_; i++)
        opt_var_[i] = current_jt_status_[i];
    
    // assign the target pose to non-linear solver
    problem_formulation_.assignTarget(kinematic_pose_, opt_var_);
    //std::cout<<"OPTIM ik number_of_joints_ "<<number_of_joints_<<std::endl;
    double minf;
    
    nlopt::result result = nlopt::FAILURE;
    auto start_time = std::chrono::high_resolution_clock::now();

    try
    {
        result = nlopt_solver_.optimize(opt_var_, minf); 
    }
    catch(const std::exception& e)
    {}

    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    double best_minf = minf;
    std::vector<double> best_opt_var;
    best_opt_var = opt_var_;

    if( (fabs(minf) > opt_param_.opt_config.min_cost ) && (elapsed.count() < opt_param_.opt_config.max_time) )
    {
        do
        {
            for (uint i=0; i< opt_var_.size(); i++)
                opt_var_[i]=random_dist_.at(i)(random_gen_);
            try
            {
                result = nlopt_solver_.optimize(opt_var_, minf);
            }
            catch(const std::exception& e)
            {}

            finish_time = std::chrono::high_resolution_clock::now();
            elapsed = finish_time - start_time;
            //std::cout<<"Result loop "<<result<<" new minf loop="<<minf<<"  "<<elapsed.count()<<"  "<<opt_param_.max_time<<std::endl;
            if(minf < best_minf)
            {
                best_minf = minf;
                best_opt_var = opt_var_;
            }

            if(fabs(minf) <= opt_param_.opt_config.min_cost)
            {
                best_minf = minf;
                best_opt_var = opt_var_;
                break;
            }
        }while ( (elapsed.count() < opt_param_.opt_config.max_time) );
    }

    opt_var_ = best_opt_var;

    solution.resize(1);
    solution[0].resize(number_of_joints_);
    solution[0].names = jt_names_;
    for(std::size_t i = 0; i < opt_var_.size(); ++i)
    {
        solution[0].elements.at(i).position = opt_var_[i];
        solution[0].elements.at(i).speed = 0.0;
    }

    if ( result == nlopt::SUCCESS )
    {
        solver_status.statuscode = kinematics_library::KinematicsStatus::IK_FOUND;
        return true;
    }
    else if ((( result == nlopt::STOPVAL_REACHED)  || ( result == nlopt::XTOL_REACHED )) && ( best_minf <= opt_param_.opt_config.min_cost ) )
    {
        solver_status.statuscode = KinematicsStatus::APPROX_IK_SOLUTION;
        return true;
    }
    else if (( result == nlopt::MAXTIME_REACHED ) || ( result == nlopt::MAXEVAL_REACHED ))
    {
        solver_status.statuscode = kinematics_library::KinematicsStatus::IK_TIMEOUT;
        return false;
    }

    solver_status.statuscode = kinematics_library::KinematicsStatus::NO_IK_SOLUTION;
    return false;

}

bool OptSolver::solveFK (const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status )
{
    getKinematicJoints ( kdl_chain_, joint_angles, jt_names_, current_jt_status_ );

    convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );

    if ( fk_kdlsolver_pos_->JntToCart ( kdl_jt_array_, kdl_frame_ ) >= 0 ) 
    {
        kdlToRbs ( kdl_frame_, fk_pose );
        solver_status.statuscode = KinematicsStatus::FK_FOUND;
        fk_pose.sourceFrame = kinematic_pose_.sourceFrame;
        fk_pose.targetFrame = kinematic_pose_.targetFrame;
        return true;
    }

    solver_status.statuscode = KinematicsStatus::NO_FK_SOLUTION;
    return false;
}

bool OptSolver::initialiseProblem(const ProblemParameters &problem_param)
{
    nlopt_solver_ = nlopt::opt(nlopt::LD_SLSQP, number_of_joints_);

    // lower and upper limits
    std::vector< double > lower_limits, upper_limits;
    getJointLimits( jts_limits_, lower_limits, upper_limits);

    nlopt_solver_.set_upper_bounds(upper_limits);
    nlopt_solver_.set_lower_bounds(lower_limits);

    nlopt_solver_.set_maxtime(problem_param.opt_config.max_time);

    nlopt_solver_.set_xtol_abs(problem_param.opt_config.abs_tol);
    nlopt_solver_.set_xtol_rel(problem_param.opt_config.rel_tol);

    // objectives
    nlopt_solver_.set_min_objective(objectives, this);
    // in-equality constraint - upper and lower limits
    std::vector<double> tolerance(number_of_joints_ * 2, 0.00001);
    nlopt_solver_.add_inequality_mconstraint(jointsLimitsConstraint, this, tolerance);

    opt_var_.resize(number_of_joints_);

    // random generator 
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    // std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    // std::mt19937 gen1;
    random_gen_ = std::mt19937(rd());
    
    // joint limit distributor
    random_dist_.resize(number_of_joints_);
    for(size_t i = 0; i < number_of_joints_; i++)
        random_dist_.at(i) = std::uniform_real_distribution<double>(lower_limits[i], upper_limits[i]);

    // initialise the problem formulation.
    // It sets the parameters for different costs
    return (problem_formulation_.initialise(problem_param, kdl_kinematic_chain_, jts_limits_));   
}

void OptSolver::getJointLimits(const std::vector<std::pair<double, double> > jts_limits, 
                               std::vector< double > &lower_limits, std::vector< double > &upper_limits)
{
    lower_limits.clear();
    upper_limits.clear();

    for(auto it = jts_limits.begin(); it != jts_limits.end(); it++)
    {
        lower_limits.push_back(it->first)  ;
        upper_limits.push_back(it->second)  ;
    }
    assert(lower_limits.size() == jts_limits.size());
}
}
