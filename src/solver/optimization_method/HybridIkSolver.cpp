#include "kinematics_library/HandleKinematicConfig.hpp"

namespace kinematics_library
{

double redundantObjectives(const std::vector<double>& x, std::vector<double>& grad, void* data)
{   
    HybridIkSolver *c = (HybridIkSolver *) data;
    //std::cout<<"Opt. Param X = "<<x[0]<<"  "<<x[1]<<"  "<<x[2]<<std::endl;
    // first calculate the gradient for the opt. param
    c->calculateJointGradient(x);    
    // calculate the forward kinematics and the gradient's forward kinematics for all the redundant chain
    c->calculateFK(x);    
    // calculate the inverse kinematics
    c->ik_cost_             = c->calculateOverallIKCost(x, grad);
    // calculate the joint movement cost. This cost make sure that the active chain has less joints movement
    //c->joints_mov_cost_    = c->jointMovementCost(x, grad);
    // calculate the overall costs
    c->overall_costs_       =  c->ik_cost_ + c->joints_mov_cost_;
    //std::cout<<"Overall Cost = "<<c->overall_costs_<<". ik cost = "<<c->ik_cost_<<" : joints_mov_cost = "<<c->joints_mov_cost_<<"\n"<<std::endl;
    //std::cout<<"Overall Cost = "<<c->overall_costs_<<std::endl;
    return c->overall_costs_;
}

//double jointsLimitsConstraint(const std::vector<double>& x, std::vector<double>& grad, void* data)
void redundantJointsLimitsConstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
{
    // m = number of constraints
    // n = size of x    
    HybridIkSolver *c = (HybridIkSolver *) data;
    c->getJointsLimitsConstraintCost( m, n, x, result, grad);       
}

HybridIkSolver::HybridIkSolver (const std::vector<std::pair<double, double> > &jts_limits, const KDL::Tree &kdl_tree, 
                                const KDL::Chain &kdl_kinematic_chain, const KDL::Chain &kdl_chain )
{
    ik_cost_= 0.0;
    joints_mov_cost_ = 0.0;
    overall_costs_ = 0.0;

    kdl_tree_  = kdl_tree;
    kdl_chain_ = kdl_chain;
    kdl_kinematic_chain_ = kdl_kinematic_chain;
}

HybridIkSolver::~HybridIkSolver()
{
    if(fk_kdlsolver_pos_)
    {
        delete fk_kdlsolver_pos_;
        fk_kdlsolver_pos_ = NULL;
    }

    if(node_chain_fk_solver_)
    {
        delete node_chain_fk_solver_;
        node_chain_fk_solver_= NULL;
    }

    if(passive_chain_fk_solver_)
    {
        delete passive_chain_fk_solver_;
        passive_chain_fk_solver_ = NULL;
    }
}

bool HybridIkSolver::loadKinematicConfig( const kinematics_library::KinematicsConfig &kinematics_config, 
                                        kinematics_library::KinematicsStatus &kinematics_status)
{
    // assign the config
    YAML::Node input_config;
    // check whether the config could be loaded or not.
    if(!handle_kinematic_config::loadConfigFile(kinematics_config.solver_config_abs_path, kinematics_config.solver_config_filename, input_config))
    {
        LOG_ERROR("[HybridIkSolver]: Unable to load kinematic config file %s from %s", kinematics_config.solver_config_filename.c_str(), 
                    kinematics_config.solver_config_abs_path.c_str());
        kinematics_status.statuscode = kinematics_library::KinematicsStatus::NO_CONFIG_FILE;
        return false;
    }

    // get the optimization related config
    const YAML::Node& opt_config_node = input_config["opt_config"];
    
    if(!handle_kinematic_config::getOptParamConfig(opt_config_node, opt_param_))
    {
        LOG_ERROR("[HybridIkSolver]: Unable to read opimization config from kinematic config file");
        kinematics_status.statuscode = kinematics_library::KinematicsStatus::CONFIG_READ_ERROR;        
        return false;
    }

    // get the active and passive chain weight
    const YAML::Node& cost_config_node = input_config["cost_config"];
    
    if(!handle_kinematic_config::getCostsWeightConfig(cost_config_node, chain_costs_weight_))
    {
        LOG_ERROR("[HybridIkSolver]: Unable to read chain cost weights from kinematic config file");
        kinematics_status.statuscode = kinematics_library::KinematicsStatus::CONFIG_READ_ERROR;        
        return false;
    }

    // get passive and active chain    
    kinematics_library::KinematicsFactory kinematics_factory;
    kinematics_library::KinematicsConfig passive_chain_config, active_chain_config;

    const YAML::Node& kinematics_config_node = input_config["passive_chain_config"];

    if( !handle_kinematic_config::getPassiveChainConfig(kinematics_config_node, passive_chain_config))
    {
        LOG_ERROR("[HybridIkSolver]: Unable to read kinematic config for passive chain");
        kinematics_status.statuscode = kinematics_library::KinematicsStatus::CONFIG_READ_ERROR;
        return false;
    }
    // Need to find a nice way to handle this abs path
    passive_chain_config.solver_config_abs_path = kinematics_config.solver_config_abs_path;
    passive_chain_config.urdf_file = kinematics_config.urdf_file;

    passive_chain_kin_solver_ = nullptr;
    passive_chain_kin_solver_ = kinematics_factory.getKinematicsSolver (passive_chain_config, kinematics_status );
    if(!passive_chain_kin_solver_)
    {
        LOG_INFO("Cannot get the kinematic solver for the redundant chain");
        return false;
    }

    // now the active chain
    const YAML::Node& active_chain_node = input_config["active_chain_config"];

    if( !handle_kinematic_config::getActiveChainConfig(active_chain_node, active_chain_config))
    {
        LOG_ERROR("[HybridIkSolver]: Unable to read kinematic config for active chain");
        kinematics_status.statuscode = kinematics_library::KinematicsStatus::CONFIG_READ_ERROR;
        return false;
    }
    active_chain_config.solver_config_abs_path  = kinematics_config.solver_config_abs_path;
    active_chain_config.urdf_file               = kinematics_config.urdf_file;

    if ( !kinematics_factory.initialise ( active_chain_config, kinematics_status ) )
        return false;    

    LOG_INFO_S<<"[KinematicsFactory]: HybridIK solver is selected";    
    active_chain_               = kinematics_factory.getKDLChain();
    active_chain_jts_limits_    = kinematics_factory.getJointLimits();
    num_active_joints_          = active_chain_.getNrOfJoints();

    if(!initialiseSolver ( kinematics_config, kinematics_status ))
        return false;

    kinematics_status.statuscode = kinematics_library::KinematicsStatus::SUCCESS;

    return true;
}

bool HybridIkSolver::initialiseSolver ( const KinematicsConfig &kinematics_config, 
                                        kinematics_library::KinematicsStatus &kinematics_status )
{
    // fk solver
    fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_chain_ );
    assignVariables ( kinematics_config, kdl_kinematic_chain_ );
    kdl_jt_array_.resize ( number_of_joints_ );

    // initialise the problem 
    initialiseProblem(opt_param_);

    // get the kinematic chain for each of the redundant chain.
    // node: base link-> active chain's base link, tip link -> passive chain's base link
    // full: base link-> passive chain's tip link,  tip link -> passive chain's tip link
    node_chain_fk_pose_grad_.resize(num_active_joints_);
    // gradients related varaibles
    passive_chain_pose_grad_.resize ( num_active_joints_ );
    passive_chain_ik_solutions_grad_.resize ( num_active_joints_ );

    // joint status of the redundant chain    
    // get the node chain
    getKinematicChain( kdl_tree_, kinematics_config.base_name, passive_chain_kin_solver_->getKinematicChainBaseName(), 
                        kinematics_status, node_chain_);

    // create kdl based fk solver for each chain
    node_chain_fk_solver_ = new KDL::ChainFkSolverPos_recursive ( node_chain_ );
    // get the passive chain
    getKinematicChain(  kdl_tree_, passive_chain_kin_solver_->getKinematicChainBaseName(), 
                        passive_chain_kin_solver_->getKinematicChainTipName(), kinematics_status, passive_chain_);
    // create kdl based fk solver for redundant chain
    passive_chain_fk_solver_ = new KDL::ChainFkSolverPos_recursive ( passive_chain_ );
    // allocating the ik solution variables
    // currently we get the first ik solution
    passive_chain_ik_solutions_.resize(1);
    passive_chain_ik_solutions_.at(0).resize(passive_chain_.getNrOfJoints());
    // get the joint names for the redundant chain
    passive_chain_joint_status_.resize(passive_chain_.getNrOfJoints());
    for ( std::size_t jn = 0; jn < passive_chain_.getNrOfJoints(); jn++ )
    {
        passive_chain_joint_status_.names.at(jn)       = passive_chain_.getSegment(jn).getJoint().getName();
        passive_chain_ik_solutions_.at(0).names.at(jn) = passive_chain_.getSegment(jn).getJoint().getName();
    }

    jump_ = std::numeric_limits<float>::epsilon();

    // random generator 
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    // std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    // std::mt19937 gen1;
    random_gen_ = std::mt19937(rd());
    
    // joint limit distributor
    random_dist_.resize(num_active_joints_);
    for(size_t i = 0; i < num_active_joints_; i++)
        random_dist_.at(i) = std::uniform_real_distribution<double>(jts_lower_limit_[i], jts_upper_limit_[i]);
        

    return true;
}

bool HybridIkSolver::getKinematicChain(  const KDL::Tree &kdl_tree, const std::string &base_name, const std::string &tip_name, 
                                            kinematics_library::KinematicsStatus &kinematics_status, KDL::Chain &kdl_chain)
{    
    if ( !kdl_tree.getChain ( base_name, tip_name, kdl_chain) )
    {
        kinematics_status.statuscode = kinematics_library::KinematicsStatus::KDL_CHAIN_FAILED;
        LOG_FATAL ( "[HybridIkSolver::getKinematicChain]: Could not initialise KDL chain from base: %s to the tip:%s!.",  
                     base_name.c_str(), tip_name.c_str());
        return false;
    }
    else
    {
        LOG_INFO ( "[HybridIkSolver::getKinematicChain]: KDL chain initialised from %s to %s with size %d",
                    base_name.c_str(), tip_name.c_str(), kdl_chain.segments.size() );
    }
    return true;
}

void HybridIkSolver:: getJointsLimitsConstraintCost( const unsigned &constraints_size, const unsigned &x_size,
                                                        const double* x, double *result, double* grad)
{
    //jt - upperlimit lessthanequal 0
    //lowerlimit - jt lessthanequal 0
    for(size_t i =0; i < x_size; i++ )
    {
        // lower limit
        result[i] = (jts_lower_limit_[i] - x[i]);
        //upper limit     
        result[i+x_size] = (x[i] - jts_upper_limit_[i]);
    }
    
    // gradient
    if (grad)
    {
        // lower limit
        for(size_t i =0; i < (constraints_size/2) ; i++ )
        {
            for(size_t j =0; j < (x_size) ; j++ )
            {
                grad[(i*x_size) + j] =  ((jts_lower_limit_[j] - jt_ang_grad_.at(i).at(j) ) - result[j]) / (2*jump_);
            }
        }
        // upper limit
        for(size_t i =constraints_size/2; i < (constraints_size) ; i++ )
        {
            for(size_t j =0; j < (x_size) ; j++ )
            {
                grad[(i*x_size)+ j] = ((jt_ang_grad_.at(i-constraints_size/2).at(j) - jts_upper_limit_[j] ) - result[j+x_size]) / (2*jump_);
            }
        }
    }
}

void HybridIkSolver::calculateJointGradient(const std::vector<double> &opt_jt_ang)
{
    for (std::size_t i=0; i< opt_jt_ang.size(); i++) 
    {
        for(std::size_t ii = 0; ii < opt_jt_ang.size(); ii++)
        {    
            jt_ang_grad_.at(i).at(ii) = opt_jt_ang[ii];
        }
        jt_ang_grad_.at(i).at(i)  = opt_jt_ang[i] + jump_;
    }
}

double HybridIkSolver::jointMovementCost(const std::vector<double> &opt_jt_ang,  std::vector<double>& grad)
{
    double cost = 0.0;
    size_t data_size = opt_jt_ang.size();
    std::vector<double> cost_vec(data_size);

    for(size_t i = 0; i < opt_jt_ang.size(); i++)
    {
        cost_vec[i] = fabs(opt_jt_ang[i] * chain_costs_weight_.joint_movement);
        cost += cost_vec[i] ;
    }

    if(!grad.empty())
    {
        std::vector<double> x_rollout(data_size);

        for (std::size_t i=0; i<data_size; i++) 
        {
            grad[i] +=  (((fabs(jt_ang_grad_.at(i).at(i))* chain_costs_weight_.joint_movement)- cost_vec[i]) / (2.0*jump_));            
        }
    }
    return cost;
}

void HybridIkSolver::calculateFK(const std::vector<double> &opt_jt_ang)
{       
    // calculate the fk for the node chain
    calculateNodeChainFK(opt_jt_ang, node_chain_fk_pose_, passive_chain_pose_);
    
    // calculate the fk for the chain - gradient        
    calculateNodeChainFKGrad(opt_jt_ang, node_chain_fk_pose_grad_, passive_chain_pose_grad_);
}

void HybridIkSolver::calculateNodeChainFK(const std::vector<double> &opt_jt_ang, KDL::Frame &node_pose, KDL::Frame &passive_pose)
{
    // fk active chain
    KDL::JntArray kdl_jt_array;
    kdl_jt_array.resize(opt_jt_ang.size());
    kinematics_library::convertVectorToKDLArray(opt_jt_ang, kdl_jt_array);
    
    if(node_chain_fk_solver_->JntToCart(kdl_jt_array, node_pose) < 0)
    {
        LOG_FATAL("[calculateFK]: Cannot able to get the forward kinematics");
        return;
    }    
    // calculate the redundant chain tip pose
    passive_pose = node_pose.Inverse() * passive_full_chain_pose_;
}

void HybridIkSolver::calculateNodeChainFKGrad(  const std::vector<double> &opt_jt_ang, KDLFrameVec &node_pose_grad, 
                                                    KDLFrameVec &passive_pose_grad )
{
    size_t data_size = opt_jt_ang.size();
    std::vector<double> x_rollout(data_size);

    for (std::size_t i=0; i<data_size; i++) 
    {
        // calculate the tip pose of node chain
        calculateNodeChainFK(jt_ang_grad_.at(i), node_pose_grad.at(i), passive_pose_grad.at(i));
    }
}

double HybridIkSolver::positionCost( const KDL::Frame& target_pose, const KDL::Frame& node_pose, const kinematics_library::AbstractKinematicPtr &kinematic_solver, 
                                        const base::samples::Joints &joint_status, kinematics_library::KinematicsStatus &solver_status)
{
    base::samples::RigidBodyState fk_pose;
    if(!kinematic_solver->solveFK (joint_status, fk_pose, solver_status))
    {
        exit(0);
        return 0;
    }    
    KDL::Frame fk_pose_kdl;
    kinematics_library::rbsToKdl(fk_pose, fk_pose_kdl);      

    return (target_pose.p - (node_pose*fk_pose_kdl).p).Norm();
}

double HybridIkSolver::positionCost( const KDL::Frame& target_pose, const KDL::Frame& node_pose)
{
    return ((target_pose.p - node_pose.p).Norm());
}

double HybridIkSolver::calculateIKCost(const KDL::Frame &chain_pose, const KDL::Frame &node_pose, IkSolutions &ik_solution)
{    
    double ikcost = 0;
    base::samples::RigidBodyState kinematic_pose;

    kinematics_library::KinematicsStatus solver_status;
    // convert the kdl frame to RBS
    kinematics_library::kdlToRbs(chain_pose, kinematic_pose);
    // calculate the IK
    kinematic_pose.sourceFrame = passive_chain_kin_solver_->getKinematicChainBaseName();
    kinematic_pose.targetFrame = passive_chain_kin_solver_->getKinematicChainTipName();
    
    if(!passive_chain_kin_solver_->solveIK(kinematic_pose, passive_chain_joint_status_, ik_solution, solver_status)) 
    {
        double position_cost = positionCost( passive_full_chain_pose_, node_pose);
        ikcost += (chain_costs_weight_.position * position_cost);        
    }
    else
    {        
        ikcost = chain_costs_weight_.ik;
        for(int kk=0; kk < ik_solution[0].names.size();kk++)
        {
            if(std::isnan(ik_solution[0].elements[kk].position))
            {
                std::cout<<"NAN "<<ik_solution[0].names.at(kk).c_str()<<"  "<<ik_solution[0].elements[kk].position<<std::endl;
                exit(0);
            }
        }
    }
    
    return ikcost;
}

double HybridIkSolver::calculateOverallIKCost(const std::vector<double>& opt_jt_ang, std::vector<double>& grad)
{
    double ikcost = calculateIKCost( passive_chain_pose_, node_chain_fk_pose_, passive_chain_ik_solutions_);
    size_t data_size = opt_jt_ang.size();
    if(!grad.empty())
    {
        for (std::size_t i = 0; i < data_size; i++) 
        {   
            double tt = calculateIKCost( passive_chain_pose_grad_.at(i), node_chain_fk_pose_grad_.at(i), passive_chain_ik_solutions_grad_.at(i));            
            grad[i] = (((tt- ikcost) / (2.0*jump_)));            
        }
    }
    return ikcost;
}


void HybridIkSolver::convertPoseBetweenDifferentFramesFK( const KDL::Tree &kdl_tree, const base::samples::Joints &joint_status, 
                                        const base::samples::RigidBodyState &source_pose, base::samples::RigidBodyState &target_pose)
{    

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

        kinematics_library::rbsToKdl(target_pose, calculated_frame);

        KDL::Chain new_chain;

        if(!kdl_tree.getChain(target_pose.sourceFrame.c_str() , source_pose.sourceFrame.c_str() , new_chain))
        {
            LOG_FATAL("[KinematicsHelper]: Could not get KDL transformation chain between base_link: %s to tip_link: %s", 
                        target_pose.sourceFrame.c_str(), source_pose.sourceFrame.c_str());
            exit(1);
        }
        else
        LOG_DEBUG("[KinematicsHelper]: KDL transformation chain initilised");

        KDL::ChainFkSolverPos_recursive fk  ( new_chain);
        KDL::Frame fk_pose;
        int ct = 0;
        for(unsigned int i = 0; i < new_chain.segments.size(); i++)
        {
            if ( ! ( new_chain.getSegment (i).getJoint().getType() == KDL::Joint::None ) )
            ct++;
        }

        KDL::JntArray kdl_jt_array;
        kdl_jt_array.resize ( ct );
        int jt_ct=0;

        for(unsigned int i = 0; i < new_chain.segments.size(); i++)
        {            
            if ( ! ( new_chain.getSegment (i).getJoint().getType() == KDL::Joint::None ) )
            {
                kdl_jt_array.data(jt_ct) = joint_status[new_chain.getSegment(i).getJoint().getName()].position;
                jt_ct++;
            }
        }

        if(fk.JntToCart(kdl_jt_array, fk_pose) < 0)
        {        
            LOG_FATAL("[calculateFK]: Cannot able to get the forward kinematics");exit(1);
            return;
        }

        new_frame = fk_pose * calculated_frame;
        kinematics_library::kdlToRbs(new_frame, target_pose);
        
    }

    if( (source_pose.targetFrame.compare(target_pose.targetFrame) != 0) && (!target_pose.targetFrame.empty()) )
    {
        LOG_DEBUG("[RobotKinematics]: Kinematic tipname = %s and input tipname = %s are not the same", 
        target_pose.targetFrame.c_str(), source_pose.targetFrame.c_str());

        // transform_tip_kt_  -> transformation from kinematic tip to target tip
        KDL::Frame calculated_frame, new_frame;	    
        KDL::Frame transform_tip_kt;
        transform_tip_kt.Identity();
        
        KDL::Chain new_chain;

        if(!kdl_tree.getChain(source_pose.targetFrame.c_str() , target_pose.targetFrame.c_str() , new_chain))
        {
            LOG_FATAL("[KinematicsHelper]: Could not get KDL transformation chain between base_link: %s to tip_link: %s", 
                        source_pose.targetFrame.c_str(), target_pose.targetFrame.c_str());
            exit(1);
        }
        else
        LOG_DEBUG("[KinematicsHelper]: KDL transformation chain initilised");

        KDL::ChainFkSolverPos_recursive fk ( new_chain);
        KDL::Frame fk_pose;
        KDL::JntArray kdl_jt_array;
        int ct = 0;
        for(unsigned int i = 0; i < new_chain.segments.size(); i++)
        {
            if ( ! ( new_chain.getSegment (i).getJoint().getType() == KDL::Joint::None ) )
            ct++;
                
        }

        kdl_jt_array.resize ( ct);

        for(unsigned int i = 0; i < new_chain.segments.size(); i++)
        {
            if ( ! ( new_chain.getSegment (i).getJoint().getType() == KDL::Joint::None ) )
                kdl_jt_array.data(i) = joint_status[new_chain.getSegment(i).getJoint().getName()].position;
        }


        if(fk.JntToCart(kdl_jt_array, fk_pose) < 0)
        {        
            LOG_FATAL("[calculateFK]: Cannot able to get the forward kinematics");exit(1);
            return;
        }         
        
        kinematics_library::rbsToKdl(target_pose, calculated_frame);
        new_frame = calculated_frame * transform_tip_kt;
        kinematics_library::kdlToRbs(new_frame, target_pose);
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


bool HybridIkSolver::solveIK (const base::samples::RigidBodyState &target_pose, 
                            const base::samples::Joints &joint_status,
                            std::vector<base::commands::Joints> &solution,
                            kinematics_library::KinematicsStatus &solver_status )
{

    if(!convertPoseBetweenDifferentFrames(kdl_tree_, joint_status, target_pose, kinematic_pose_))
    {
        solver_status.statuscode = KinematicsStatus::KDL_CHAIN_FAILED;
        return false;
    }

    // reset the costs to zero
    ik_cost_ = 0.0; joints_mov_cost_ = 0.0; overall_costs_ = 0.0;

    // Get the target poses for the passive chain.
    KDL::JntArray active_jt_array, passive_jt_array;
    active_jt_array.resize(active_chain_.getNrOfJoints()); // before its uses joint status
    
    for(unsigned int chain_jt = 0; chain_jt < active_chain_.segments.size(); ++chain_jt)
    {        
        active_jt_array.data(chain_jt) = joint_status[active_chain_.getSegment(chain_jt).getJoint().getName()].position;
        //std::cout<<"Joint name: "<<active_chain_.getSegment(chain_jt).getJoint().getName().c_str()<<" = "<<active_jt_array.data(chain_jt)<<std::endl;            
    }

    // get the node chain pose
    if(node_chain_fk_solver_->JntToCart(active_jt_array, node_chain_fk_pose_) < 0)
    {        
        LOG_FATAL("[calculateFK]: Cannot able to get the forward kinematics");
        return false;
    }

    passive_jt_array.resize(passive_chain_.getNrOfJoints());
    for(size_t j = 0; j < passive_chain_.getNrOfJoints(); j++)
    {
        passive_jt_array.data(j) = joint_status[passive_chain_.getSegment(j).getJoint().getName()].position;
        passive_chain_joint_status_.elements.at(j).position = passive_jt_array.data(j);
        passive_chain_ik_solutions_[0].elements.at(j).position = passive_jt_array.data(j);
    }

    base::samples::RigidBodyState desired_pose_4_passive;
    
    desired_pose_4_passive.sourceFrame = passive_chain_kin_solver_->getKinematicChainBaseName();
    desired_pose_4_passive.targetFrame = passive_chain_kin_solver_->getKinematicChainTipName();

    //convertPoseBetweenDifferentFramesFK(kdl_tree_, joint_status, kinematic_pose_, desired_pose);
    if(!convertPoseBetweenDifferentFrames(kdl_tree_, joint_status, kinematic_pose_, desired_pose_4_passive))
    {
        solver_status.statuscode = KinematicsStatus::KDL_CHAIN_FAILED;
        return false;
    }
    
    kinematics_library::rbsToKdl(desired_pose_4_passive, passive_chain_target_pose_);
    //std::cout<<" Active hcain "<<desired_pose.sourceFrame.c_str()<<"  "<<desired_pose.targetFrame.c_str()<<"  "<<desired_pose.position<<std::endl;
    //std::cout<<" target pose "<<target_pose.sourceFrame.c_str()<<"  "<<target_pose.targetFrame.c_str()<<"  "<<target_pose.position<<std::endl;

    // Get the target pose of redundant chain
    passive_full_chain_pose_ = node_chain_fk_pose_ * passive_chain_target_pose_;
    //std::cout<<" passive_full_chain_pose_ "<<passive_full_chain_pose_.p[0]<<"  "<<passive_full_chain_pose_.p[1]<<"  "<<passive_full_chain_pose_.p[2]<<std::endl;
    //std::cout<<" node_chain_fk_pose_ "<<node_chain_fk_pose_.p[0]<<"  "<<node_chain_fk_pose_.p[1]<<"  "<<node_chain_fk_pose_.p[2]<<std::endl;
    //std::cout<<" passive_chain_target_pose_ "<<passive_chain_target_pose_.p[0]<<"  "<<passive_chain_target_pose_.p[1]<<"  "<<passive_chain_target_pose_.p[2]<<std::endl;
    

    // Optimize the active chain
    // get the proper frame for the respecting chain based on the tip frame.
    //convertPoseBetweenDifferentFrames ( full_kdl_tree_, target_pose, kinematic_pose_ );
    // get the joint status only for the kinematic chain that this solver intended to solve.
    kinematics_library::getKinematicJoints ( active_chain_, joint_status, jt_names_, current_jt_status_ );
    // assign the current joint angles as the optimization variables
    for(std::size_t i = 0; i < num_active_joints_; i++)
        opt_var_[i] = current_jt_status_[i];
    // get the cost
    // 1. correct passive chain ik solution
    // 2. maintain other passive chain pose
    // 3. Maybe smoothness, self-collision, etc. 

    double minf;
    
    nlopt::result result = nlopt::FAILURE;
    auto start_time = std::chrono::high_resolution_clock::now();

    try
    {
        result = nlopt_solver_.optimize(opt_var_, minf); 
    }
    catch(const std::exception& e)
    {}
    //std::cout<<"\nResult "<<result<<" minf="<<minf<<std::endl;
    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    double best_minf = minf;
    std::vector<double> best_opt_var;
    best_opt_var = opt_var_;

    if( (fabs(minf) > opt_param_.min_cost ) && (elapsed.count() < opt_param_.max_time) )
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

            if(fabs(minf) <= opt_param_.min_cost)
            {
                best_minf = minf;
                best_opt_var = opt_var_;
                break;
            }
        }while ( (elapsed.count() < opt_param_.max_time) );
    }
  
    opt_var_ = best_opt_var;

    solution.clear();
    solution.resize(1);
    // assign the active chain
    for(std::size_t i = 0; i < opt_var_.size(); ++i)
    {
        base::JointState joint_state = base::JointState::Position(opt_var_[i]);
        solution[0].elements.push_back(joint_state);
        solution[0].names.push_back(jt_names_.at(i));        
        if(std::isnan(opt_var_[i]))
        {
            std::cout<<"NAN "<<jt_names_.at(i).c_str()<<"  "<<opt_var_[i]<<std::endl;
            exit(0);
        }
    }
    
    // assign the redundant chain
    base::samples::Joints passive_solution = passive_chain_ik_solutions_[0];
    for(std::size_t j = 0; j < passive_solution.size(); ++j)
    {
        base::JointState joint_state = base::JointState::Position(passive_solution.elements.at(j).position);
        solution[0].elements.push_back(joint_state);
        solution[0].names.push_back(passive_solution.names.at(j));
        //std::cout<<passive_solution.names.at(j).c_str()<<"  "<<joint_state.position<<std::endl;  //*57.2958
        if(std::isnan(passive_solution.elements.at(j).position))
        {
            std::cout<<"NAN "<<passive_solution.names.at(j).c_str()<<"  "<<passive_solution.elements.at(j).position<<std::endl;
            exit(0);
        }
    }

    if ( result == nlopt::SUCCESS )
    {
        solver_status.statuscode = kinematics_library::KinematicsStatus::IK_FOUND;
        //std::cout<<"[MULT_IK]: IK FOUND"<<std::endl;
        return true;
    }
    else if ((( result == nlopt::STOPVAL_REACHED)  || ( result == nlopt::XTOL_REACHED )) && ( best_minf <= opt_param_.min_cost ) )
    {
        solver_status.statuscode = KinematicsStatus::APPROX_IK_SOLUTION;
        return true;
    }
    else if (( result == nlopt::MAXTIME_REACHED ) || ( result == nlopt::MAXEVAL_REACHED ))
    {
        solver_status.statuscode = kinematics_library::KinematicsStatus::IK_TIMEOUT;
        //std::cout<<"[MULT_IK]: IK_TIMEOUT"<<std::endl;
        return false;
    }
    else
    {
        solver_status.statuscode = kinematics_library::KinematicsStatus::NO_IK_SOLUTION;
        //std::cout<<"[MULT_IK]: NO_IK_SOLUTION"<<std::endl;
        return false;
    }
}

bool HybridIkSolver::solveFK (const base::samples::Joints &joint_angles, base::samples::RigidBodyState &fk_pose, 
                              kinematics_library::KinematicsStatus &solver_status )
{
    kinematics_library::getKinematicJoints ( kdl_kinematic_chain_, joint_angles, jt_names_, current_jt_status_ );

    kinematics_library::convertVectorToKDLArray ( current_jt_status_, kdl_jt_array_ );
    
    if ( fk_kdlsolver_pos_->JntToCart ( kdl_jt_array_, kdl_frame_ ) >= 0 ) 
    {
        kinematics_library::kdlToRbs ( kdl_frame_, fk_pose );
        //std::cout<<kinematic_pose_.sourceFrame.c_str()<<"  "<<kinematic_pose_.targetFrame.c_str()<<"  "<<kinematic_pose_.position<<std::endl;
        solver_status.statuscode = kinematics_library::KinematicsStatus::FK_FOUND;
        fk_pose.sourceFrame = kinematic_pose_.sourceFrame;
        fk_pose.targetFrame = kinematic_pose_.targetFrame;        
        return true;
    }
    solver_status.statuscode = kinematics_library::KinematicsStatus::NO_FK_SOLUTION;
    return false;
}

void HybridIkSolver::initialiseProblem(const OptParamConfig &problem_param)
{
    nlopt_solver_ = nlopt::opt(nlopt::LD_SLSQP, num_active_joints_);

    // lower and upper limits
    getJointLimits( active_chain_jts_limits_, jts_lower_limit_, jts_upper_limit_);

    nlopt_solver_.set_upper_bounds(jts_upper_limit_);
    nlopt_solver_.set_lower_bounds(jts_lower_limit_);

    nlopt_solver_.set_maxtime(problem_param.max_time);

    nlopt_solver_.set_xtol_abs(problem_param.abs_tol);
    nlopt_solver_.set_xtol_rel(problem_param.rel_tol);

    //nlopt_solver_.set_stopval(0.015);

    // objectives
    nlopt_solver_.set_min_objective(redundantObjectives, this);
    // in-equality constraint - upper and lower limits
    std::vector<double> tolerance(num_active_joints_ * 2, 0.00001); 
           
    nlopt_solver_.add_inequality_mconstraint(redundantJointsLimitsConstraint, this, tolerance);

    opt_var_.resize(num_active_joints_);

    // initialise joint angles gradient
    jt_ang_grad_.resize(num_active_joints_);
    for(size_t i = 0; i < num_active_joints_; i++)
    {
        jt_ang_grad_.at(i).resize(num_active_joints_);
    }
  
}

void HybridIkSolver::getJointLimits(const std::vector<std::pair<double, double> > jts_limits, 
                               std::vector< double > &lower_limits, std::vector< double > &upper_limits)
{
    lower_limits.clear();
    upper_limits.clear();
    
    for(auto it = jts_limits.begin(); it != jts_limits.end(); it++)
    {
        lower_limits.push_back((it->first));
        upper_limits.push_back((it->second));        
    }
    assert(lower_limits.size() == jts_limits.size());
}
}