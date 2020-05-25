#ifndef _IKFASTSOLVER_HPP_
#define _IKFASTSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include "kinematics_library/abstract/AbstractKinematics.hpp"
#include "kinematics_library/abstract/KinematicsHelper.hpp"
#include "kinematics_library/abstract/Ikfast.h"

#include <dlfcn.h>

/** \file AbstractIkFastSolver.hpp
*    \brief Interface to use ikfast generated Inverse solver.
*/

namespace kinematics_library
{
/**
* @struct redundant_ik_solution
* @brief This struct contains redundant inverse solution.
*/
struct redundant_ik_solution
{
    std::vector<double> ik_sol;
    double opt_iksol_index_value;
};

struct compare_result
{
    bool operator()(redundant_ik_solution const &a, redundant_ik_solution const &b) {
     return a.opt_iksol_index_value < b.opt_iksol_index_value;
     }

};

struct IkFastConfig
{
    IkFastConfig() : ikfast_lib(""), use_current_value_as_free_joint_param(false){}
    // ikfast shared library absolute path
    std::string ikfast_lib;
    // joint weight. It is used for finding the weighted optimal inverse kinematic solution, 
    // when the inverse solver gave more than one solution.
    std::vector <double> joints_weight;
    // use the current value as free joint param. CAUTION: Index need to match
    bool use_current_value_as_free_joint_param;
    // use this value as free joint parameter
    std::vector <double> free_joint_param;
};


/**
 * @class IkFastsolver
 * @brief Kinematics solvers using IkFast.
 */
class IkFastSolver : public AbstractKinematics
{

public:
    /**
    * @brief  constructor
    */
    IkFastSolver(const std::vector<std::pair<double, double> > jts_limits, const KDL::Tree &kdl_tree, 
                 const KDL::Chain &_kdl_chain);
    /**
    * @brief  destructor
    */
    virtual ~IkFastSolver();

    bool loadKinematicConfig( const KinematicsConfig &kinematics_config, KinematicsStatus &kinematics_status);

    /**
    * @brief Calculate the joint angles for a manipulator to reach a desired Pose.
    * @param base_link Target Position is given respect to this link
    * @param target_position Desired position for the target link
    * @param target_orientation Desired orientation for the target link in quaternion(based on euler ZYX)
    * @param joint_status Contains current joint angles.
    * @param solution Inverse solution
    * @param solver_status Solution status or error code
    * @return true if a inverse solution was found or else return false
    */
    bool solveIK(const base::samples::RigidBodyState target_pose, const base::samples::Joints &joint_status,
                 std::vector<base::commands::Joints> &solution, KinematicsStatus &solver_status);

    /**
    * @brief Calculate pose of a manipulator given its joint angles    
    * @param joint_angles joint angles of the manipulator
    * @param fk_position fk position
    * @param fk_orientation fk orienation in quaternion
    * @param solver_status Solution status or error code
    * @return true if a forward solution was found or else return false
    */
    bool solveFK(const base::samples::Joints &joint_angles,
                 base::samples::RigidBodyState &fk_pose,
                 KinematicsStatus &solver_status);
    
    void getChainSegementPose(const base::samples::Joints &joint_angles,  std::vector<KDL::Frame> &segement_pose){}

private:
    /**
    * @brief Pick an optimal from all possible solution.
    *        The solution is checked for joint limits and
    *        a solution nearer to current joint status is picked.
    * @param cur_jtang current joint angles of the manipulator.
    * @param redundantSolutions list holding redundant solution.
    * @param optSol vector of optimal solutions.
    * @return true if optimal solution found.
    */
    bool pickOptimalIkSolution(const std::vector<double> &cur_jtang,
                               const ikfast::IkSolutionList<IkReal> &redundantSolutions,
                               std::vector<std::vector<double> > &optSol);

    /**
    * @brief Check joint limits.
    * @param redundantSolutions list holding redundant solution.
    * @param all_solution all possible solutions.
    * @param joint_limit_exceed_solution_index index representing joint limits exceed solution.
    * @param jt_limit_exceed_sol_ct joint limit counter.
    */
    bool checkJointLimits(  const ikfast::IkSolutionList<IkReal> &redundantSolutions,
                            std::vector<std::vector<double> > &all_solution,
                            std::vector<int> &joint_limit_exceed_solution_index,
                            int &jt_limit_exceed_sol_ct);

    bool getIKFASTFunctionPtr(const std::string ikfast_lib, KinematicsStatus &kinematics_status);
    
    IkFastConfig ikfast_config_;    
    std::vector<std::pair<double,double> > jts_limits_;
    ikfast::IkSolutionList<IkReal> ik_solutions_;
    std::vector<IkReal> vfree_;
    int *freeparams_;
    void *ikfast_handle_;

    bool (*computeIkFn)(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions);
    void (*computeFkFn)(const IkReal* j, IkReal* eetrans, IkReal* eerot);
    int (*getNumFreeParametersFn)();
    int *(*getFreeParametersFn)();
    
};


}

#endif
