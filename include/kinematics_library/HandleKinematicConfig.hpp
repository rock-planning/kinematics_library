#ifndef HANDLEKINEMATICCONFIG_HPP_
#define HANDLEKINEMATICCONFIG_HPP_

#include <string>
#include <fstream>
#include <iostream>
#include <ostream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include "kinematics_library/abstract/IkFastSolver.hpp"
#include "kinematics_library/abstract/KdlSolver.hpp"
#if TRAC_IK_LIB_FOUND    
    #include "kinematics_library/abstract/TracIkSolver.hpp"  
#endif
#if OPT_LIB_FOUND    
    #include "kinematics_library/solver/optimization_method/OptSolver.hpp" 
    #include "kinematics_library/solver/optimization_method/HybridIkSolver.hpp"
#endif
#include "kinematics_library/solver/shimizu_method/SRSKinematicSolver.hpp"
#include "kinematics_library/solver/asfour_method/IK7DoFSolver.hpp"
#include "kinematics_library/solver/optimization_method/OptConfig.hpp"


namespace handle_kinematic_config
{
    
inline bool loadConfigFile(const std::string &filepath, const std::string &filename, YAML::Node &config)
{
    std::stringstream config_file;
    config_file << filepath << "/" << filename;

    try
    {
        // YAML::LoadFile is not throwing any exception if the file doesn't exist.
        // std::iftream is used here in order to check the input file existence.
        std::ifstream input_file(config_file.str().c_str());
        if(input_file)
        {
            input_file.close();
            config = YAML::LoadFile(config_file.str());
        }
        else
            return false;
    }
    catch (YAML::ParserException& e)
    {
        LOG_ERROR("[loadConfigFile]: Error at loading the YAML File. The error is : %s", e.what() );
        return false;        
    }
    
    return true;
}

template<typename T>
bool getValue (const YAML::Node &yaml_data, std::string name, T &value)
{
    if (const YAML::Node data = yaml_data[name])
    {
        value = data.as<T>();
        return true;
    }
    else
        LOG_INFO("[getValue]: Key %s doesn't exist", name.c_str());

    return false; 
}

template<typename T>
bool getValue (const YAML::Node &yaml_data, std::string name, const T& df, T &value)
{
    if (const YAML::Node data = yaml_data[name])
    {
        value = data.as<T>();
    }
    else
        value = df;

    return true;

}

bool getKinematicsConfig(const YAML::Node &yaml_data, kinematics_library::KinematicsConfig &config);

bool getIkFastConfig(const std::string &dir_path, const YAML::Node &yaml_data, kinematics_library::IkFastConfig &config);

bool getKdlConfig(const YAML::Node &yaml_data, kinematics_library::KdlConfig &config);

#if(TRAC_IK_LIB_FOUND)
    bool getTracIkConfig(const YAML::Node &yaml_data, kinematics_library::TracIkConfig &config);
#endif

#if(OPT_LIB_FOUND)
    bool getOptIkGrooveParam(const YAML::Node &yaml_data, const std::string &cost_name, kinematics_library::GrooveVariable &groove_var);
    bool getOptIKConfig(const YAML::Node &yaml_data, kinematics_library::ProblemParameters &config);
    bool getOptParamConfig(const YAML::Node &yaml_data, kinematics_library::OptParamConfig &config);
    bool getCostsWeightConfig(const YAML::Node &yaml_data, kinematics_library::CostsWeight &weight);
    bool getPassiveChainConfig(const YAML::Node &yaml_data, kinematics_library::KinematicsConfig &config);
    bool getActiveChainConfig(const YAML::Node &yaml_data, kinematics_library::KinematicsConfig &config);
#endif


bool getSRSConfig(const YAML::Node &yaml_data, kinematics_library::SRSKinematicConfig &config);

bool getIK7DoFConfig(const YAML::Node &yaml_data, kinematics_library::IK7DoFConfig &config);

bool getDHParamConfig(const YAML::Node& yaml_data, kinematics_library::DHParamConfig &config);

}
#endif
