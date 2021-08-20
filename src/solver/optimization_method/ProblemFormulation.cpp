#include "kinematics_library/solver/optimization_method/ProblemFormulation.hpp"

using namespace kinematics_library;

ProblemFormulation::ProblemFormulation( )
{
}

ProblemFormulation::~ProblemFormulation( )
{
    if ( fk_kdlsolver_pos_ ) 
    {
        fk_kdlsolver_pos_ = NULL;
        delete fk_kdlsolver_pos_;
    }
}

bool ProblemFormulation::initialise(const ProblemParameters &problem_param, const KDL::Chain &kdl_kinematic_chain, 
                                    const std::vector<std::pair<double, double> > &jts_limits)
{   
    problem_param_ = problem_param;
    jts_limits_ = jts_limits;
    kdl_kinematic_chain_ = kdl_kinematic_chain;
    
    //initialise the groove parameter
    for( std::map<CostType, GrooveVariable >::iterator it = problem_param_.groove_param.begin(); 
         it != problem_param_.groove_param.end(); it++)
    {
        switch(it->first)
        {
            case POSITION_COST:
            {
                pos_groove_var_ = it->second; break;
            }
            case ORIENTATION_COST:
            {
                ort_groove_var_ = it->second; break;            
            }
            case VELOCITY_COST:
            {
                vel_groove_var_ = it->second; break;
            }
            case ACCELERATION_COST:
            {
                acc_groove_var_ = it->second; break;
            }
            case JERK_COST:
            {
                jerk_groove_var_ = it->second; break;
            }
            case MIN_VEL_EE_COST:
            {
                min_vel_groove_var_ = it->second; break;
            }
            default:
            {
                std::cout<<"No config"<<std::endl;
                exit(0);
            }
        }
    }
    //initialise the cost weight
    for( std::map<CostType, double >::iterator it = problem_param_.costs_weight.begin(); 
         it != problem_param_.costs_weight.end(); it++)
    {
        switch(it->first)
        { 
            case POSITION_COST:
            {
                pos_cost_weight_ = it->second; break;
            }
            case ORIENTATION_COST:
            {
                ort_cost_weight_ = it->second; break;            
            }
            case VELOCITY_COST:
            {
                vel_cost_weight_ = it->second; break;
            }
            case ACCELERATION_COST:
            {
                acc_cost_weight_ = it->second; break;
            }
            case JERK_COST:
            {
                jerk_cost_weight_ = it->second; break;
            }
            default:
            {
                std::cout<<"No config"<<std::endl;
                exit(0);
            }
        }
    }

    opt_var_size_ = jts_limits_.size();

    fk_jac_.resize(opt_var_size_);
    jtang_jac_.resize(opt_var_size_);
    for(size_t i = 0; i < opt_var_size_; i++)
    {
        jtang_jac_.at(i).resize(opt_var_size_);
    }

    // variable to store the previous joint value, which will be used to calculate velocity, acc., and jerk.
    prev_jtang_.resize(opt_var_size_);
    for(size_t i = 0; i < prev_jtang_.size(); i++)
        prev_jtang_.at(i).resize(4);


    // initialise the joint velocity and acceleration
    jt_vel_(opt_var_size_, 1);
    jt_acc_(opt_var_size_, 1);

    jump_ = std::numeric_limits<float>::epsilon();

    jts_lower_limit_.resize(opt_var_size_);
    jts_upper_limit_.resize(opt_var_size_);   

    for(size_t i = 0; i < opt_var_size_;i ++)
    {
        jts_lower_limit_.at(i) = jts_limits_.at(i).first;
        jts_upper_limit_.at(i) = jts_limits_.at(i).second;
    }

    // forward kinematic using KDL
    kdl_jt_array_.resize ( opt_var_size_ );
    fk_kdlsolver_pos_ = new KDL::ChainFkSolverPos_recursive ( kdl_kinematic_chain_ );    

    return true;
}

void ProblemFormulation::calculateFK(const std::vector<double> &opt_jt_ang, KDL::JntArray &kdl_jt_array, KDL::Frame &kdl_frame)
{
    convertVectorToKDLArray(opt_jt_ang, kdl_jt_array);

    if(fk_kdlsolver_pos_->JntToCart(kdl_jt_array, kdl_frame) < 0)    
        LOG_FATAL("[calculateFK]: Cannot able to get the forward kinematics");
}

void ProblemFormulation::assingGoal(const KDL::Frame &frame)
{
    target_frame_ = frame;
}

void ProblemFormulation::assignTarget(const base::samples::RigidBodyState &target_pose, const std::vector<double> &cur_jts)
{
    //target_pose_ = target_pose;
    kinematics_library::rbsToKdl(target_pose, target_pose_);

    // now we assign the previous joint value as current joint value
    for(size_t i = 0; i < cur_jts.size(); i++)
    {
        for(size_t j = 0; j < 4; j++)
            prev_jtang_[i][j] = cur_jts[i];
    }
}

void ProblemFormulation::calculateDerivatives(const std::vector<double>& x)
{    
    jt_vel_ = getDerivative(VELOCITY, x);
    jt_acc_ = getDerivative(ACCELERATION, x);
}

double ProblemFormulation::getOverallCost(const std::vector<double>& x, std::vector<double>& grad)
{
    //   for(size_t i = 0; i < x.size(); i++)
    //       std::cout<<x[i]<<"  ";
    //   std::cout<<std::endl;

    assert(fk_jac_.size() == x.size());

    // get the End-Effector pose: forward kinematic
    calculateFK(x, kdl_jt_array_, kdl_frame_);
    // calculate the jacobian 
    calculateJacobian(x);

    // get the joint velocity and joint acceleration
    calculateDerivatives(x);    

    // position cost
    double position_cost        = getPosistionCost(kdl_frame_, grad);
    double orientation_cost     = getOrientationCost(kdl_frame_, grad);
    //double velocity_cost        = getVelocityCost(x, grad);
    //double acceleration_cost    = getAccelerationCost(x, grad);
    //double jerk_cost            = getJerkCost(x, grad);

    storePreviousRobotState(x);
    //std::cout<<"Cost ="<<position_cost <<"  "<<orientation_cost<<"  "<<velocity_cost<<"  "<<acceleration_cost<<"  "<<jerk_cost<<std::endl;
    //std::cout<<"Cost ="<<position_cost <<"  "<<orientation_cost<<std::endl;
    //return ( pos_cost_weight_*position_cost + ort_cost_weight_*orientation_cost + (vel_cost_weight_*velocity_cost) + 
    //         (acc_cost_weight_*acceleration_cost) + (jerk_cost_weight_*jerk_cost));
    return ( pos_cost_weight_*position_cost + ort_cost_weight_*orientation_cost );
}

void ProblemFormulation::getJointsLimitsConstraintCost( const unsigned &constraints_size, const unsigned &x_size,
                                                        const double* x, double *result, double* grad)
{
    
    for(size_t i =0; i < x_size; i++ )
    {
        // lower limit
        result[i] = (jts_lower_limit_[i] - x[i]);
        //upper limit     
        result[i+x_size] = (x[i] - jts_upper_limit_[i]);
           //std::cout<<result[i]<<"   "<<result[i+x_size]<<"  "<<x(i,0)<<std::endl;
    }

    // gradient
    if (grad)
    {
        // lower limit
        for(size_t i =0; i < (constraints_size/2) ; i++ )
        {
            for(size_t j =0; j < (x_size) ; j++ )
            {
                grad[(i*x_size) + j] =  ((jts_lower_limit_[j] - jtang_jac_.at(i).at(j) ) - result[j]) / (2*jump_);
            }
        }
        
        // upper limit
        for(size_t i =constraints_size/2; i < (constraints_size) ; i++ )
        {
            for(size_t j =0; j < (x_size) ; j++ )
            {
                grad[(i*x_size)+ j] = ((jtang_jac_.at(i-constraints_size/2).at(j) - jts_upper_limit_[j] ) - result[j+x_size]) / (2*jump_);
            }
        }
        
    }
    //std::cout<<std::endl;
}

void ProblemFormulation::storePreviousRobotState(const std::vector<double>& x)
{
    for( size_t i = 0; i < x.size(); i++)
    {    
        prev_jtang_[i][3] = prev_jtang_[i][2];
        prev_jtang_[i][2] = prev_jtang_[i][1];
        prev_jtang_[i][1] = prev_jtang_[i][0];
        prev_jtang_[i][0] = x[i];
    }

}

void ProblemFormulation::calculateJacobian(const std::vector<double>& x)
{ 
    size_t data_size = x.size();

    std::vector<double> x_rollout(data_size);

    for (std::size_t i=0; i<data_size; i++) 
    {
        for(std::size_t ii = 0; ii < data_size; ii++)
        {        
            x_rollout.at(ii)        = x[ii];
            jtang_jac_.at(i).at(ii) = x[ii];
        }
        x_rollout.at(i)         = x[i] + jump_;
        jtang_jac_.at(i).at(i)  = x[i] + jump_;

        calculateFK(x_rollout, kdl_jt_array_, fk_jac_.at(i));        
    }
    
}

double ProblemFormulation::getPosistionCost(const KDL::Frame& fk_pose, std::vector<double>& grad)
{
    double position_cost = (target_pose_.p - fk_pose.p).Norm();
    if(!grad.empty())
    {        
        for (std::size_t i = 0; i < fk_jac_.size(); i++) 
        {            
            //grad[i] = ((target_pose_.position - fk_jac_[i].position).norm() - position_cost) / (2.0*jump);

            double new_cost_1 = ((target_pose_.p - fk_jac_[i].p).Norm() - position_cost) / (2.0*jump_);
            
            grad[i] = grooveDerivativeFunction(pos_groove_var_, new_cost_1);
        }
    }

    // return a normalised cost using the groove function
    return grooveFunction(pos_groove_var_, position_cost);
}

double ProblemFormulation::getOrientationCost(const KDL::Frame& fk_pose, std::vector<double>& grad)
{
    double cost_1 = getQuaternionDiff(target_pose_.M, fk_pose.M);
    //Eigen::Quaterniond minus_fk_ort(-fk_pose.w(), -fk_pose.x(), -fk_pose.y(), -fk_pose.z());

    Eigen::Quaterniond quad;
    fk_pose.M.GetQuaternion(quad.x(), quad.y(), quad.z(), quad.w());
    
    double cost_2 = getQuaternionDiff(target_pose_.M, KDL::Rotation::Quaternion(-quad.x(), -quad.y(), -quad.z(), -quad.w()));

    double cost = std::min(cost_1, cost_2);
        
    if(!grad.empty())
    {        
        for (std::size_t i = 0; i < fk_jac_.size(); i++) 
        { 
            // double dervi_cost = getQuaternionDiff(target_pose_.orientation, fk_jac_[i].orientation);       
            // grad[i] += ((dervi_cost - cost) / (2.0*jump) );            
    
            double dervi_cost = getQuaternionDiff(target_pose_.M, fk_jac_[i].M);
            double new_cost_1 = ((dervi_cost - cost) / (2.0*jump_) );
            grad[i] += grooveDerivativeFunction(ort_groove_var_, new_cost_1);
        }
    }

    // return a normalised cost using the groove function
    return grooveFunction(ort_groove_var_, cost);    
}



double ProblemFormulation::getVelocityCost(const std::vector<double>& x, std::vector<double>& grad)
{
    //double cost = getDerivative(VELOCITY, x).norm();
    double cost = jt_vel_.norm();
    
    //std::cout<<" cost "<<cost;

    if(!grad.empty())
    {
        for (std::size_t i=0; i< opt_var_size_; i++)
        {
            double grad_cost = (cost - (getDerivative(VELOCITY, jtang_jac_.at(i)).norm())) / (2.0*jump_);
            //std::cout<<" "<<grooveDerivativeFunction(vel_groove_var_, grad_cost);
            
            grad[i] += (0.0001*grooveDerivativeFunction(vel_groove_var_, grad_cost));
        }
    }
    // return a normalised cost using the groove function
    //std::cout<<"  "<<grooveFunction(vel_groove_var_, cost)<<std::endl;
    return grooveFunction(vel_groove_var_, cost);
}

double ProblemFormulation::getAccelerationCost(const std::vector<double>& x, std::vector<double>& grad)
{
    double cost =  jt_acc_.norm();

    if(!grad.empty())
    {
        for (std::size_t i=0; i< opt_var_size_; i++)
        {
            double grad_cost = (cost - (getDerivative(ACCELERATION, jtang_jac_.at(i)).norm())) / (2.0*jump_);            
            grad[i] += (0.0001 * grooveDerivativeFunction(acc_groove_var_, grad_cost));
        }
    }
    // return a normalised cost using the groove function
    return grooveFunction(acc_groove_var_, cost);
}

double ProblemFormulation::getJerkCost(const std::vector<double>& x, std::vector<double>& grad)
{
    double cost =  getDerivative(JERK, x).norm();

    if(!grad.empty())
    {
        for (std::size_t i=0; i< opt_var_size_; i++)
        {
            double grad_cost = (cost - (getDerivative(JERK, jtang_jac_.at(i)).norm())) / (2.0*jump_);            
            grad[i] += (0.0001 * grooveDerivativeFunction(jerk_groove_var_, grad_cost));
        }
    }
    // return a normalised cost using the groove function
    return grooveFunction(jerk_groove_var_, cost);
}

Eigen::MatrixXd ProblemFormulation::getDerivative(const DerivType &deriv_type, const std::vector<double>& x)
{
    size_t type = 0;

    switch(deriv_type)
    {        
        case VELOCITY:
        {
            type = 0; break;
        }
        case ACCELERATION:
        {
            type = 1; break;            
        }
        case JERK:
        {
            type = 2; break;
            
        }
        default:
        {
            std::cout<<"No config"<<std::endl; exit(0);
        }
    }

    Eigen::MatrixXd deriv_value(x.size(),1);

    for(size_t i = 0; i < x.size(); i++)
    {
        deriv_value(i,0) = 0.01*((DIFF_RULES[type][0] * x[i]) + (DIFF_RULES[type][1] * prev_jtang_[i][0]) + (DIFF_RULES[type][2] * prev_jtang_[i][1]) + 
                         (DIFF_RULES[type][3] * prev_jtang_[i][2]) + (DIFF_RULES[type][4] * prev_jtang_[i][3] )); 
    }

    return deriv_value;
}

double ProblemFormulation::getMinEEVelCost(const std::vector<double>& x, std::vector<double>& grad)
{
    double cost;

    // return a normalised cost using the groove function
    return grooveFunction(min_vel_groove_var_, cost);
}
double ProblemFormulation::grooveFunction(const GrooveVariable &groove_var, const double &cost)
{
    double a = cost - groove_var.s;
    return ( (-exp ( (-pow(a, 2)) / (2.0*pow(groove_var.c, 2))) ) + 
             (groove_var.r * pow(a, 2)) );

}

double ProblemFormulation::grooveDerivativeFunction(const GrooveVariable &groove_var, const double &cost)
{
    double a = cost - groove_var.s;
    double b = groove_var.c * groove_var.c;
    return ( ((a*(exp ( (-pow(a, 2)) / (2.0*b)) )) / b) + 
             (2*groove_var.r * (pow(a, 1)) ));

}



