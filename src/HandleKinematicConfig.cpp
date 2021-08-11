#include "kinematics_library/HandleKinematicConfig.hpp"

#if(TRAC_IK_LIB_FOUND)
namespace YAML {
template<>
struct convert<kinematics_library::TracIKSolverType> 
{
    static Node encode(const kinematics_library::TracIKSolverType& solver_type) 
    {
        Node node;
        node.push_back(solver_type);   
        return node;
    }

    static bool decode(const Node& node, kinematics_library::TracIKSolverType& solver_type) 
    {

        if(!node.IsScalar())
            return false;  

        std::unordered_map<std::string, kinematics_library::TracIKSolverType> stringToenum;
        stringToenum.insert({":SPEED", kinematics_library::SPEED});
        stringToenum.insert({":DISTANCE", kinematics_library::DISTANCE});
        stringToenum.insert({":MANIP1", kinematics_library::MANIP1});
        stringToenum.insert({":MANIP2", kinematics_library::MANIP2});

        LOG_INFO_S<<"[getTracIkConfig]: Selected solver Type = "<<stringToenum.at(node.Scalar());
        solver_type  = stringToenum.at(node.Scalar());
        return true;
    }
};
}
#endif


namespace YAML 
{
    template<>
    struct convert<kinematics_library::KinematicSolver> 
    {
        static Node encode(const kinematics_library::KinematicSolver& solver) 
        {
            Node node;
            node.push_back(solver);   
            return node;
        }
        
        static bool decode(const Node& node, kinematics_library::KinematicSolver& solver) 
        {
            
            if(!node.IsScalar())
                return false;  

            std::unordered_map<std::string, kinematics_library::KinematicSolver> stringToenum;
            stringToenum.insert({":IKFAST", kinematics_library::IKFAST});
            stringToenum.insert({":SRS", kinematics_library::SRS});
            stringToenum.insert({":KDL", kinematics_library::KDL});
            stringToenum.insert({":TRACIK", kinematics_library::TRACIK});
            stringToenum.insert({":IK7DOF", kinematics_library::IK7DOF});
            stringToenum.insert({":OPT", kinematics_library::OPT});
            stringToenum.insert({":HYBRIDIK", kinematics_library::HYBRIDIK});
            
            LOG_INFO_S<<"[getKinematicSolverConfig]: Selected solver Type = "<<stringToenum.at(node.Scalar());
            solver  = stringToenum.at(node.Scalar());
            return true;
        }
    };


    template<>
    struct convert<kinematics_library::ZE_MODE> 
    {
        static Node encode(const kinematics_library::ZE_MODE& ze_mode) 
        {
            Node node;
            node.push_back(ze_mode);   
            return node;
        }
        
        static bool decode(const Node& node, kinematics_library::ZE_MODE& ze_mode) 
        {
            
            if(!node.IsScalar())
                return false;  
            
            std::unordered_map<std::string, kinematics_library::ZE_MODE> stringToenum;
            stringToenum.insert({":AUTO_ZSTABLE", kinematics_library::AUTO_ZSTABLE});
            stringToenum.insert({":AUTO_ZMAX", kinematics_library::AUTO_ZMAX});
            stringToenum.insert({":AUTO_YMAX", kinematics_library::AUTO_YMAX});
            stringToenum.insert({":MANUAL", kinematics_library::MANUAL});
            
            LOG_INFO_S<<"[getIK7DoFConfig]: Selected ZE Mode Type = "<<stringToenum.at(node.Scalar());
            ze_mode  = stringToenum.at(node.Scalar());
            return true;
        }
    };
}

namespace handle_kinematic_config
{

bool getKinematicsConfig(const YAML::Node &yaml_data, kinematics_library::KinematicsConfig &config)
{

    if( (!handle_kinematic_config::getValue<std::string>(yaml_data, "config_name", config.config_name)) ||
        (!handle_kinematic_config::getValue<std::string>(yaml_data, "base_name", config.base_name)) ||
        (!handle_kinematic_config::getValue<std::string>(yaml_data, "tip_name", config.tip_name)) ||
        (!handle_kinematic_config::getValue<kinematics_library::KinematicSolver>(yaml_data, "kinematic_solver", config.kinematic_solver)) ||
        (!handle_kinematic_config::getValue<std::string>(yaml_data, "solver_config_abs_path", config.solver_config_abs_path)) ||
        (!handle_kinematic_config::getValue<std::string>(yaml_data, "solver_config_filename", config.solver_config_filename)) ||
        (!handle_kinematic_config::getValue<std::string>(yaml_data, "urdf_file", config.urdf_file)) ||
        (!handle_kinematic_config::getValue<bool>(yaml_data, "linear_relative_movement", config.linear_relative_movement)) )
        return false;

    if(config.linear_relative_movement)
    {
        if( (!handle_kinematic_config::getValue<double>(yaml_data, "interpolation_velocity", config.linear_movement_config.interpolation_velocity)) ||
            (!handle_kinematic_config::getValue<double>(yaml_data, "sampling_time", config.linear_movement_config.sampling_time)) ||
            (!handle_kinematic_config::getValue<double>(yaml_data, "position_tolerance_in_m", config.linear_movement_config.position_tolerance_in_m)) )
            return false;
    }

    return true;
}


bool getIkFastConfig(const std::string &dir_path, const YAML::Node &yaml_data, kinematics_library::IkFastConfig &config)
{
    std::string filename;
    if (!handle_kinematic_config::getValue<std::string>(yaml_data, "ikfast_lib", filename))
        return false;

    std::stringstream config_file;
    config_file << dir_path << "/" << filename;
    config.ikfast_lib = config_file.str();
    
    if (const YAML::Node &joints_weight_node    = yaml_data["joints_weight"])
    {
        config.joints_weight.resize(joints_weight_node.size());
        for(std::size_t i = 0; i < joints_weight_node.size(); i++)
            config.joints_weight.at(i) = joints_weight_node[i].as<double>();
    }
    else
    {
        LOG_WARN("[getIkFastConfig]: Key %s doesn't exist", "joints_weight");
        return false;
    }
    
    if (!handle_kinematic_config::getValue<bool>(yaml_data, "use_current_value_as_free_joint_param", config.use_current_value_as_free_joint_param))
        return false;

    if (const YAML::Node &free_joint_param_node    = yaml_data["free_joint_param"])
    {
        config.free_joint_param.resize(free_joint_param_node.size());
        for(std::size_t i = 0; i < free_joint_param_node.size(); i++)
            config.free_joint_param.at(i) = free_joint_param_node[i].as<double>();
    }
    else
    {
        LOG_WARN("[getIkFastConfig]: Key %s doesn't exist", "free_joint_param");
        return false;
    }
    return true;
}

bool getKdlConfig(const YAML::Node &yaml_data, kinematics_library::KdlConfig &config)
{
    if ((!handle_kinematic_config::getValue<unsigned int>(yaml_data, "max_iteration", config.max_iteration)) || 
        (!handle_kinematic_config::getValue<double>(yaml_data, "eps", config.eps)))
        return false;

    return true;
}

#if(TRAC_IK_LIB_FOUND)
bool getTracIkConfig(const YAML::Node &yaml_data, kinematics_library::TracIkConfig &config)
{        
    if ((!handle_kinematic_config::getValue<kinematics_library::TracIKSolverType>(yaml_data, "solver_type", config.solver_type)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "timeout_sec", config.timeout_sec)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "eps", config.eps)))
        return false;
    
    const YAML::Node &tolerances_node    = yaml_data["tolerances"];
    config.tolerances.resize(tolerances_node.size());
    for(std::size_t i = 0; i < tolerances_node.size(); i++)
        config.tolerances.at(i) = tolerances_node[i].as<double>();
    
    const YAML::Node &joints_err_weight_node    = yaml_data["joints_err_weight"];
    config.joints_err_weight.resize(joints_err_weight_node.size());
    for(std::size_t i = 0; i < joints_err_weight_node.size(); i++)
        config.joints_err_weight.at(i) = joints_err_weight_node[i].as<double>();

    return true;
}
#endif


#if(OPT_LIB_FOUND)

bool getOptIkGrooveParamAndWeight(  const YAML::Node &yaml_data, const std::string &cost_name, const kinematics_library::CostType &cost_type, 
                                    kinematics_library::ProblemParameters &config )
{
    if (const YAML::Node &cost_node = yaml_data[cost_name])
    {
        if (cost_node.size() == 4)
        { 
            kinematics_library::GrooveVariable groove_var;
            groove_var.s = cost_node[0].as<double>(); groove_var.c = cost_node[1].as<double>(); groove_var.r = cost_node[2].as<double>();            
            config.groove_param.insert ( std::pair<kinematics_library::CostType, kinematics_library::GrooveVariable>(cost_type, groove_var) );
            double weight =  cost_node[3].as<double>();
            config.costs_weight.insert ( std::pair<kinematics_library::CostType, double>(cost_type, weight) );  
        }
        else
        {
            LOG_WARN("[getOptIkGrooveParamAndWeight]: For key %s incorrect data is given", cost_name.c_str());
            return false;
        }
    }
    else
    {
        LOG_WARN("[getOptIkGrooveParamAndWeight]: Key %s doesn't exist", cost_name.c_str());
        return false;
    }

    return true;
}

bool getOptIKConfig(const YAML::Node &yaml_data, kinematics_library::ProblemParameters &config)
{           
    if( (!getOptIkGrooveParamAndWeight(yaml_data, "position_cost", kinematics_library::POSITION_COST, config))    ||
        (!getOptIkGrooveParamAndWeight(yaml_data, "orientation_cost", kinematics_library::ORIENTATION_COST, config)) ||
        (!getOptIkGrooveParamAndWeight(yaml_data, "velocity_cost", kinematics_library::VELOCITY_COST, config)) ||
        (!getOptIkGrooveParamAndWeight(yaml_data, "acceleration_cost", kinematics_library::ACCELERATION_COST, config)) ||
        (!getOptIkGrooveParamAndWeight(yaml_data, "jerk_cost", kinematics_library::JERK_COST, config)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "max_time", config.max_time)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "max_iter", config.max_iter)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "abs_tol", config.abs_tol)) )
            return false;
    
    return true;
}

bool getCostsWeightConfig(const YAML::Node &yaml_data, const std::string &chain, kinematics_library::CostsWeight &weight)
{   
    if( (!handle_kinematic_config::getValue<double>(yaml_data, chain+"ik_cost_weight", weight.ik)) ||        
        (!handle_kinematic_config::getValue<double>(yaml_data, chain+"position_cost_weight", weight.position)) )
        return false;   

    return true;   
}

bool getOptParamConfig(const YAML::Node &yaml_data, kinematics_library::OptParamConfig &config)
{    
    if( (!handle_kinematic_config::getValue<double>(yaml_data, "joint_movement_cost_weight", config.joint_movement_weight)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "max_time", config.max_time)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "max_iter", config.max_iter)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "abs_tol", config.abs_tol)) )
        return false;

    return true;        
}
#endif

bool getSRSConfig(const YAML::Node &yaml_data, kinematics_library::SRSKinematicConfig &config)
{
    if((!getDHParamConfig(yaml_data, config.dh_param)) || 
       (!handle_kinematic_config::getValue<bool>(yaml_data, "save_psi", config.save_psi)) ||
       (!handle_kinematic_config::getValue<std::string>(yaml_data, "save_psi_path", config.save_psi_path)))
       return true;
    return true;  
}

bool getIK7DoFConfig(const YAML::Node& yaml_data, kinematics_library::IK7DoFConfig &config)
{        
    if ((!handle_kinematic_config::getValue<kinematics_library::ZE_MODE>(yaml_data, "ze_mode", config.ze_mode)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "offset_base_shoulder", config.offset_base_shoulder)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "offset_shoulder_elbow", config.offset_shoulder_elbow)) ||
        (!handle_kinematic_config::getValue<double>(yaml_data, "offset_elbow_wrist", config.offset_elbow_wrist)) || 
        (!handle_kinematic_config::getValue<double>(yaml_data, "offset_wrist_tool", config.offset_wrist_tool)) ||
        (!handle_kinematic_config::getValue<std::vector<double>>(yaml_data, "theta_offsets", config.theta_offsets)) ||
        (!handle_kinematic_config::getValue<std::vector<double>>(yaml_data, "link_twists", config.link_twists)) ||
        (!handle_kinematic_config::getValue<std::vector<int>>(yaml_data, "joints_mapping", config.joints_mapping)))
        return false;

    config.joint_names              = yaml_data["joint_names"].as<std::vector<std::string>>();  
    
    return true;
}

bool getDHParamConfig(const YAML::Node& yaml_data, kinematics_library::DHParamConfig &config)
{        
    if ((!handle_kinematic_config::getValue<std::vector<double>>(yaml_data, "link_offsets", config.link_offsets)) ||
        (!handle_kinematic_config::getValue<std::vector<double>>(yaml_data, "theta_offsets", config.theta_offsets)) ||
        (!handle_kinematic_config::getValue<std::vector<double>>(yaml_data, "link_twists", config.link_twists)) ||
        (!handle_kinematic_config::getValue<std::vector<int>>(yaml_data, "joints_mapping", config.joints_mapping)))
        return false;

    config.joint_names              = yaml_data["joint_names"].as<std::vector<std::string>>();  
    
    return true;
}



}
