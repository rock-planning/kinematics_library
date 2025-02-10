#include <kinematics_library/KinematicsFactory.hpp>

using namespace kinematics_library;

kinematics_library::KinematicsConfig getKinematicsConfig(std::string test_folder_path)
{
    kinematics_library::KinematicsConfig config;

    config.config_name = "kuka_arm";
    config.base_name = "IIWA14_BASE_LINK_link";
    // config.base_name = "WEBOTS_WORLD_link";
    config.tip_name = "IIWA14_LINK_7_link";
    config.urdf_file = test_folder_path + "./data/eu-rise/eurise_scene.urdf";
    // kinematics_library::KDL kinematics_library::TRACIK kinematics_library::OPT
    config.kinematic_solver = kinematics_library::KDL;
    config.solver_config_abs_path = test_folder_path + "./config";
    // kdl_config.yml trac_ik_config.yml opt_ik_config.yml
    config.solver_config_filename = "kdl_config.yml";

    return config;
}

base::samples::Joints convertToBaseJoints(const std::vector<double> &data)
{
    base::samples::Joints joint_values;
    joint_values.names = {"IIWA14_LINK_1_joint", "IIWA14_LINK_2_joint", "IIWA14_LINK_3_joint", "IIWA14_LINK_4_joint", "IIWA14_LINK_5_joint", "IIWA14_LINK_6_joint", "IIWA14_LINK_7_joint"};
    joint_values.elements.resize(joint_values.names.size());
    assert(joint_values.size() == data.size());
    for (size_t i = 0; i < data.size(); i++)
        joint_values.elements[i].position = data[i];

    return joint_values;
}

void printFKPose(const base::samples::RigidBodyState &fk_pose)
{
    std::cout << "Source Frame = " << fk_pose.sourceFrame.c_str() << std::endl;
    std::cout << "Target Frame = " << fk_pose.targetFrame.c_str() << std::endl;

    std::cout << "Position: ";
    for (size_t i = 0; i < fk_pose.position.size(); i++)
        std::cout << fk_pose.position(i) << "  ";

    std::cout << std::endl;

    std::cout << "Orientation: " << fk_pose.orientation.x() << "  " << fk_pose.orientation.y() << "  " << fk_pose.orientation.z() << "  " << fk_pose.orientation.w() << std::endl;
}

void printIKSolutions(const std::vector<base::commands::Joints> &ik_solns)
{

    for (size_t i = 0; i < ik_solns.size(); i++)
    {
        for (size_t j = 0; j < ik_solns[i].elements.size(); j++)
        {
            std::cout << ik_solns[i].elements[j].position << ", ";
        }
        std::cout << std::endl;
    }
}

void printKinemticsStatus(KinematicsStatus &kinematics_status)
{
    switch (kinematics_status.statuscode)
    {
    case kinematics_library::KinematicsStatus::KDL_TREE_FAILED:
        std::cout << "KDL_TREE_FAILED" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::KDL_CHAIN_FAILED:
        std::cout << "KDL_CHAIN_FAILED" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::URDF_FAILED:
        std::cout << "URDF_FAILED" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND:
        std::cout << "NO_KINEMATIC_SOLVER_FOUND" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::IK_FOUND:
        std::cout << "IK_FOUND" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::NO_IK_SOLUTION:
        std::cout << "NO_IK_SOLUTION" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::NO_FK_SOLUTION:
        std::cout << "NO_FK_SOLUTION" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::IK_TIMEOUT:
        std::cout << "IK_TIMEOUT" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::IK_JOINTLIMITS_VIOLATED:
        std::cout << "IK_JOINTLIMITS_VIOLATED" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::NO_CONFIG_FILE:
        std::cout << "NO_KINEMATIC_CONFIG_FILE" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::CONFIG_READ_ERROR:
        std::cout << "KINEMATIC_CONFIG_READ_ERROR" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::INVALID_STATE:
        std::cout << "INVALID_KINEMATIC_STATE" << std::endl;
        break;
    case kinematics_library::KinematicsStatus::APPROX_IK_SOLUTION:
        std::cout << "IK_FOUND" << std::endl;
        break;
    default:
    {
        std::cout << "unknown Kinematics state" << kinematics_status.statuscode << std::endl;
        throw new std::runtime_error("This kinematic status is unknown");
        break;
    }
    }
}

int main(int argc, char *argv[])
{
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    std::cout << "!             Test function fot testing kinematics library       !\n";
    std::cout << "!              ./test_kuka absolute_path_to_test_folder          !\n";
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n";
    if (argc != 2)
    {
        std::cout << "The test function expect the absolute path to the test folder" << std::endl;
        return 0;
    }

    std::string test_folder_path = argv[1];
    std::cout << "The given absolute path: " << test_folder_path.c_str() << std::endl;

    // get kinematics config
    kinematics_library::KinematicsConfig config = getKinematicsConfig(test_folder_path);

    // create kinematics library object
    kinematics_library::KinematicsFactory kinematics_factory;
    KinematicsStatus kinematics_status;

    kinematics_library::AbstractKinematicPtr robot_kinematics = kinematics_factory.getKinematicsSolver(config, kinematics_status);
    if (robot_kinematics == NULL)
    {
        std::cout << "Cannot create kinematics library object. Refer to kinematics status to get the error information \n";
        printKinemticsStatus(kinematics_status);
        return 0;
    }

    base::samples::RigidBodyState fk_pose;

    // calculate forward kinematics for the following joint values
    std::vector<double> fk_vec_values = {1.95265, 1.92175, -1.97375, -0.515773, 2.85604, 1.13886, -1.19884};
    base::samples::Joints fk_joint_values = convertToBaseJoints(fk_vec_values);

    if (!robot_kinematics->solveFK(fk_joint_values, fk_pose, kinematics_status))
    {
        std::cout << "Cannot find forward kinematics. Refer to kinematics status to get the error information" << std::endl;
        printKinemticsStatus(kinematics_status);
    }

    printFKPose(fk_pose);

    std::cout << "\n\n!!!!!!!! Check the inverse kinematics  !!!!!!!! \n";

    // calculate inverse kinematics for the above calculated FK pose
    std::vector<double> ik_seed_vec_values = {1.95265, 1.92175, -1.97375, -0.515773, 2.85604, 1.13886, -1.19884};
    // std::vector<double> ik_seed_vec_values = {0.5, 0.5, 1.5, 0.5, 0.5, 0.5, 0.5};
    base::samples::Joints ik_seed_joint_values = convertToBaseJoints(ik_seed_vec_values); // here we assign zero joint values as seed ik solution
    std::vector<base::commands::Joints> ik_solutions;

    fk_pose.position(0) = -0.301340;
    fk_pose.position(1) = 0.418330;
    fk_pose.position(2) = 0.490160;

    fk_pose.orientation.x() = 0.086556;
    fk_pose.orientation.y() = 0.966314;
    fk_pose.orientation.z() = -0.004447;
    fk_pose.orientation.w() = 0.242335;

    fk_pose.sourceFrame = "IIWA14_BASE_LINK_link";
    fk_pose.targetFrame = "IIWA14_LINK_7_link";

    if (robot_kinematics->solveIK(fk_pose, ik_seed_joint_values, ik_solutions, kinematics_status))
    {
        std::cout << "Found IK Solution\n";
        printIKSolutions(ik_solutions);
    }
    else
    {
        std::cout << "No IK Found. Refer to kinematics status to get the error information" << std::endl;
        printKinemticsStatus(kinematics_status);
    }

    // std::cout<<"\n\n!!!!!!!! Check the IK solution by calculation forward kinematics using the ik solution !!!!!!!!\n";
    // // calculate forward kinematics for the ik solution
    // if(!robot_kinematics->solveFK( ik_solutions[0], fk_pose, kinematics_status))
    // {
    //     std::cout<<"Cannot find forward kinematics. Refer to kinematics status to get the error information"<<std::endl;
    //     printKinemticsStatus(kinematics_status);
    // }

    // printFKPose(fk_pose);

    return 0;
}
