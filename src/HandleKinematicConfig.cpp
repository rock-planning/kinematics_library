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

kinematics_library::IkFastConfig getIkFastConfig(const std::string &dir_path, const YAML::Node &yaml_data)
{
    kinematics_library::IkFastConfig config;

    std::string filename    = handle_kinematic_config::getValue<std::string>(yaml_data, "ikfast_lib"); 

    std::stringstream config_file;
    config_file << dir_path << "/" << filename;
    config.ikfast_lib = config_file.str();
    
    const YAML::Node &joints_weight_node    = yaml_data["joints_weight"];
    config.joints_weight.resize(joints_weight_node.size());
    for(std::size_t i = 0; i < joints_weight_node.size(); i++)
        config.joints_weight.at(i) = joints_weight_node[i].as<double>();
    
    config.use_current_value_as_free_joint_param = handle_kinematic_config::getValue<bool>(yaml_data, "use_current_value_as_free_joint_param"); 
    
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

/*kinematics_library::SRSKinematicConfig getSRSConfig(const YAML::Node &yaml_data)
{
    kinematics_library::SRSKinematicConfig config;
    
    config.offset_base_shoulder     = handle_kinematic_config::getValue<double>(yaml_data, "offset_base_shoulder");
    config.offset_shoulder_elbow    = handle_kinematic_config::getValue<double>(yaml_data, "offset_shoulder_elbow");
    config.offset_elbow_wrist       = handle_kinematic_config::getValue<double>(yaml_data, "offset_elbow_wrist");
    config.offset_wrist_tool        = handle_kinematic_config::getValue<double>(yaml_data, "offset_wrist_tool");
    config.save_psi                 = handle_kinematic_config::getValue<bool>(yaml_data, "save_psi");
    config.save_psi_path            = handle_kinematic_config::getValue<std::string>(yaml_data, "save_psi_path");
    
    return config;

}*/

kinematics_library::IK7DoFConfig getIK7DoFConfig(const YAML::Node& yaml_data)
{
    kinematics_library::IK7DoFConfig config;
    
    config.ze_mode                  = handle_kinematic_config::getValue<kinematics_library::ZE_MODE>(yaml_data, "ze_mode");
    config.offset_base_shoulder     = handle_kinematic_config::getValue<double>(yaml_data, "offset_base_shoulder");
    config.offset_shoulder_elbow    = handle_kinematic_config::getValue<double>(yaml_data, "offset_shoulder_elbow");
    config.offset_elbow_wrist       = handle_kinematic_config::getValue<double>(yaml_data, "offset_elbow_wrist");
    config.offset_wrist_tool        = handle_kinematic_config::getValue<double>(yaml_data, "offset_wrist_tool");
    config.theta_offsets            = handle_kinematic_config::getValue<std::vector<double>>(yaml_data, "theta_offsets");
    config.joint_names              = yaml_data["joint_names"].as<std::vector<std::string>>();  
    
    return config;

}


}
