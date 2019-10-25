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
#include "solver/shimizu_method/SRSKinematicSolver.hpp"
#include "solver/asfour_method/IK7DoFSolver.hpp"


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
        std::cout << e.what() << "\n";
    }
    
    return true;
}

template<typename T>
T getValue (const YAML::Node &yaml_data, std::string name)
{
        T value = T();

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
        T value = T();

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

kinematics_library::SRSKinematicConfig getSRSConfig(const YAML::Node &yaml_data);    

kinematics_library::IK7DoFConfig getIK7DoFConfig(const YAML::Node &yaml_data);


}
#endif
