#ifndef _IKFASTSOLVER_HPP_
#define _IKFASTSOLVER_HPP_

#include <vector>
#include <boost/function.hpp>
#include <string>

#include "abstract/AbstractKinematics.hpp"
#include "KinematicsHelper.hpp"
#include "abstract/Ikfast.h"

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
    IkFastSolver(const std::size_t number_of_joints, const std::vector<double> jts_weight, const std::vector<std::pair<double, double> > jts_limits,
		 bool (*_ComputeIkFn)(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions),
		 void (*_ComputeFkFn)(const IkReal* j, IkReal* eetrans, IkReal* eerot));
    /**
    * @brief  destructor
    */
    virtual ~IkFastSolver();
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
    bool getIK( const std::string &base_link,
                const base::Vector3d &target_position,
                const base::Quaterniond &target_orientation,
                const std::vector<double> &joint_status,
                std::vector<double> &solution,
                KinematicsStatus &solver_status);

    /**
    * @brief Calculate pose of a manipulator given its joint angles
    * @param base_link FK will be calculated with respect to this link
    * @param target_link FK will be calculated till this target_link
    * @param joint_angles joint angles of the manipulator
    * @param fk_position fk position
    * @param fk_orientation fk orienation in quaternion
    * @param solver_status Solution status or error code
    * @return true if a forward solution was found or else return false
    */
    bool getFK( const std::string &base_link,
                const std::string &target_link,
                const std::vector<double> &joint_angles,
                base::Vector3d &fk_position,
                base::Quaterniond &fk_orientationZYX,
                KinematicsStatus &solver_status);
 

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
    bool pickOptimalIkSolution( const std::vector<double> &cur_jtang,
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
    /**
    * @brief joint weight used for checking nearest solution.
    */
    std::size_t number_of_joints;
    std::vector<double> jts_weight;
    std::vector<std::pair<double,double> > jts_limits;

    ikfast::IkSolutionList<IkReal> ik_solutions;


   bool (*ComputeIkFn_)(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions);
   void (*ComputeFkFn_)(const IkReal* j, IkReal* eetrans, IkReal* eerot);
	
};


}

#endif
