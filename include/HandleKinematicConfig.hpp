#ifndef HANDLEKINEMATICCONFIG_HPP_
#define HANDLEKINEMATICCONFIG_HPP_

#include <string>
#include <fstream>
#include <iostream>
#include <ostream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include "abstract/IkFastSolver.hpp"
#include "abstract/KdlSolver.hpp"
#if TRAC_IK_LIB_FOUND    
    #include "abstract/TracIkSolver.hpp"  
#endif



namespace handle_kinematic_config
{
    
inline void loadConfigFile(const std::string &filepath, const std::string &filename, YAML::Node &config)
{
    std::stringstream config_file;
    config_file << filepath << "/" << filename;

    try
    {
        config = YAML::LoadFile(config_file.str());
    }
    catch (YAML::ParserException& e)
    {
        std::cout << e.what() << "\n";
    }
}

template<typename T>
T getValue (const YAML::Node &yaml_data, std::string name)
{
        T value;

        if (const YAML::Node data = yaml_data[name])
        {
            value = data.as<T>();
        }
        else
            std::cout << "Key "<< name <<" doesn't exist\n";

        return value; 
}

template<typename T>
T getValue (const YAML::Node &yaml_data, std::string name, const T& df)
{
        T value;

        if (const YAML::Node data = yaml_data[name])
        {
            value = data.as<T>();
        }
        else
            value = df;

        return value;

}

kinematics_library::IkFastConfig getIkFastConfig(const std::string &dir_path, const YAML::Node &yaml_data);

kinematics_library::KdlConfig getKdlConfig(const YAML::Node &yaml_data);

#if(TRAC_IK_LIB_FOUND)
    kinematics_library::TracIkConfig getTracIkConfig(const YAML::Node &yaml_data);    
#endif


}
#endif
