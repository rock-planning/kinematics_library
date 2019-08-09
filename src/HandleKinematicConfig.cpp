#include <HandleKinematicConfig.hpp>

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

namespace handle_kinematic_config
{

kinematics_library::IkFastConfig getIkFastConfig(const YAML::Node &yaml_data)
{
    kinematics_library::IkFastConfig config;
    
    config.ikfast_lib_abs_path    = handle_kinematic_config::getValue<std::string>(yaml_data, "ikfast_lib_abs_path"); 
    
    const YAML::Node &joints_weight_node    = yaml_data["joints_weight"];
    config.joints_weight.resize(joints_weight_node.size());
    for(std::size_t i = 0; i < joints_weight_node.size(); i++)
        config.joints_weight.at(i) = joints_weight_node[i].as<double>();
    
    const YAML::Node &free_joint_param_node    = yaml_data["free_joint_param"];
    config.free_joint_param.resize(free_joint_param_node.size());
    for(std::size_t i = 0; i < free_joint_param_node.size(); i++)
        config.free_joint_param.at(i) = free_joint_param_node[i].as<double>();

    return config;
}
kinematics_library::KdlConfig getKdlConfig(const YAML::Node &yaml_data)
{
    kinematics_library::KdlConfig config;

    config.max_iteration    = handle_kinematic_config::getValue<unsigned int>(yaml_data, "max_iteration");    
    config.eps              = handle_kinematic_config::getValue<double>(yaml_data, "eps");   

    return config;
}

#if(TRAC_IK_LIB_FOUND)
kinematics_library::TracIkConfig getTracIkConfig(const YAML::Node &yaml_data)
{
    kinematics_library::TracIkConfig config;

    config.solver_type      = handle_kinematic_config::getValue<kinematics_library::TracIKSolverType>(yaml_data, "solver_type");  
    config.max_iteration    = handle_kinematic_config::getValue<unsigned int>(yaml_data, "max_iteration");    
    config.timeout_sec      = handle_kinematic_config::getValue<double>(yaml_data, "timeout_sec");
    config.eps              = handle_kinematic_config::getValue<double>(yaml_data, "eps");   
    
    const YAML::Node &tolerances_node    = yaml_data["tolerances"];
    config.tolerances.resize(tolerances_node.size());
    for(std::size_t i = 0; i < tolerances_node.size(); i++)
        config.tolerances.at(i) = tolerances_node[i].as<double>();
    
    const YAML::Node &joints_err_weight_node    = yaml_data["joints_err_weight"];
    config.joints_err_weight.resize(joints_err_weight_node.size());
    for(std::size_t i = 0; i < joints_err_weight_node.size(); i++)
        config.joints_err_weight.at(i) = joints_err_weight_node[i].as<double>();

    return config;
}
#endif

}