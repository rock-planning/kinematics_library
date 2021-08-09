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
    convertPoseBetweenDifferentFrames ( kdl_tree_, target_pose, kinematic_pose_ );
    // get the joint status only for the kinematic chain that this solver intended to solve.
    getKinematicJoints ( kdl_chain_, joint_status, jt_names_, current_jt_status_ );
    // assign the current joint angles as the optimization variables
    for(std::size_t i = 0; i < number_of_joints_; i++)
        opt_var_[i] = current_jt_status_[i];

    // assign the target pose to non-linear solver
    problem_formulation_.assignTarget(target_pose, opt_var_);

    double minf;
    
    nlopt::result result = nlopt::FAILURE;
    for(uint iter = 0; iter < opt_param_.max_iter; iter++)
    {
        result = nlopt_solver_.optimize(opt_var_, minf); 
        //std::cout<<"minf = "<<minf<<std::endl;
    }   

    //std::cout<<"Result "<<result<<" minf="<<minf<<std::endl;

    solution.resize(1);
    solution[0].resize(number_of_joints_);
    solution[0].names = jt_names_;
    for(std::size_t i = 0; i < opt_var_.size(); ++i)
    {
        solution[0].elements.at(i).position = opt_var_[i];
        solution[0].elements.at(i).speed = 0.0;
    }

    if (( result == nlopt::SUCCESS ) )
    {
        solver_status.statuscode = KinematicsStatus::IK_FOUND;
        return true;
    }
    else if ( result == nlopt::XTOL_REACHED )
    {
        solver_status.statuscode = KinematicsStatus::APPROX_IK_SOLUTION;
        return true;
    }    
    else if (( result == nlopt::MAXTIME_REACHED ) || ( result == nlopt::MAXEVAL_REACHED ))
    {
        solver_status.statuscode = KinematicsStatus::IK_TIMEOUT;
        return false;
    }
    else
    {
        solver_status.statuscode = KinematicsStatus::NO_IK_SOLUTION;
        return false;
    }
}

bool OptSolver::solveFK (const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, KinematicsStatus &solver_status )
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

bool OptSolver::initialiseProblem(const ProblemParameters &problem_param)
{
    nlopt_solver_ = nlopt::opt(nlopt::LD_SLSQP, number_of_joints_);

    // lower and upper limits
    std::vector< double > lower_limits, upper_limits;
    getJointLimits( jts_limits_, lower_limits, upper_limits);

    nlopt_solver_.set_upper_bounds(upper_limits);
    nlopt_solver_.set_lower_bounds(lower_limits);

    nlopt_solver_.set_maxtime(problem_param.max_time);

    nlopt_solver_.set_xtol_abs(problem_param.abs_tol);

    // objectives
    nlopt_solver_.set_min_objective(objectives, this);
    // in-equality constraint - upper and lower limits
    std::vector<double> tolerance(number_of_joints_ * 2, 0.00001);        
    nlopt_solver_.add_inequality_mconstraint(jointsLimitsConstraint, this, tolerance);

    opt_var_.resize(number_of_joints_);

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
