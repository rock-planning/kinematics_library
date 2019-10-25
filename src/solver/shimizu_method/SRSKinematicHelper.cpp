#include "kinematics_library/solver/shimizu_method/SRSKinematicHelper.hpp"


int complement_of_infeasible_psi( const std::vector < ArmAngle > &infeasible_psi, std::vector< ArmAngle > &complimented_infeasbile_psi)
{

    int size_infeasible_psi = 0;
    int succeeded = 0;
    ArmAngle new_range;

    size_infeasible_psi = infeasible_psi.size();

    new_range.psi.resize(2);


    for(int i = 0; i < size_infeasible_psi; i++)
    {
        if(infeasible_psi.at(i).psi.size()>1)
        {
            succeeded = -2;
            return succeeded;
        }

        new_range.joint_name = infeasible_psi.at(i).joint_name;
        new_range.joint_number = infeasible_psi.at(i).joint_number;

        if((infeasible_psi.at(i).psi.at(0).first >= -kinematics_library::PI) && (infeasible_psi.at(i).psi.at(0).second <= kinematics_library::PI))
        {                       
            new_range.psi.at(0) = std::make_pair(-kinematics_library::PI,(infeasible_psi.at(i).psi.at(0).first- kinematics_library::HALFDEGREE )); // this HALFDEGREE is explained in document
            new_range.psi.at(1) = std::make_pair((infeasible_psi.at(i).psi.at(0).second + kinematics_library::HALFDEGREE), kinematics_library::PI); // this HALFDEGREE is explained in document

            complimented_infeasbile_psi.push_back(new_range);

        }
        else if( ((infeasible_psi.at(i).psi.at(0).first >= -kinematics_library::PI) && (infeasible_psi.at(i).psi.at(0).first <= kinematics_library::PI)) &&
                 ((infeasible_psi.at(i).psi.at(0).second >= -kinematics_library::PI) && (infeasible_psi.at(i).psi.at(0).second >= kinematics_library::PI)) )
        {
            new_range.psi.at(0) = std::make_pair(-kinematics_library::PI,(infeasible_psi.at(i).psi.at(0).first- kinematics_library::HALFDEGREE )); // this HALFDEGREE is explained in document
            new_range.psi.at(1) = std::make_pair(kinematics_library::PI, kinematics_library::PI);

            complimented_infeasbile_psi.push_back(new_range);

        }
        else if( ((infeasible_psi.at(i).psi.at(0).first <= -kinematics_library::PI) && (infeasible_psi.at(i).psi.at(0).first <= kinematics_library::PI)) &&
                 ((infeasible_psi.at(i).psi.at(0).second >= -kinematics_library::PI) && (infeasible_psi.at(i).psi.at(0).second <= kinematics_library::PI)) )
        {
            new_range.psi.at(0) = std::make_pair(-kinematics_library::PI, -kinematics_library::PI);
            new_range.psi.at(1) = std::make_pair((infeasible_psi.at(i).psi.at(0).second + kinematics_library::HALFDEGREE), kinematics_library::PI); // this HALFDEGREE is explained in document

            complimented_infeasbile_psi.push_back(new_range);
        }
        else
        {
            succeeded = -1;
            return succeeded;

        }
    }

    return succeeded;
}


int union_joints_with_only_one_feasible_armangle(const std::vector< ArmAngle > &feasbile_armangle, std::vector< ArmAngle > &result)
{
    int sz_vector = feasbile_armangle.size();
    std::vector< double > range_start(sz_vector, 0.0), range_end(sz_vector, 0.0);
    int succeeded = 0;

    for(int i = 0; i < sz_vector; i++)
    {
        if(feasbile_armangle.at(i).psi.size()>1)
        {
            succeeded = -2;
            return succeeded;
        }
        range_start.at(i) = feasbile_armangle.at(i).psi.at(0).first;
        range_end.at(i) = feasbile_armangle.at(i).psi.at(0).second;
    }

    result.resize(1);
    result.at(0).psi.resize(1);
    result.at(0).joint_number = 100;
    result.at(0).joint_name = "union_single_feasible_aa";
    result.at(0).psi.at(0) = std::make_pair(*max_element(range_start.begin(), range_start.end()), *min_element(range_end.begin(), range_end.end()));

    if(result.at(0).psi.at(0).first > result.at(0).psi.at(0).second)
        succeeded = -1;

    return succeeded;
}

int union_of_all_feasible_armangle(const std::vector< ArmAngle > &unsorted_feasible_psi, std::vector< std::pair<double,double> > &final_feasible_armangle)
{
    int succeeded = 0;
    int size_unsorted_feasible_psi = unsorted_feasible_psi.size();
    std::vector< std::pair<double,double> > temp_result;
    std::pair< double,double > psi_pair;
    double case1_start = 0.0, case1_end = 0.0, case2_start = 0.0, case2_end = 0.0;

    for(size_t i = 0; i < unsorted_feasible_psi.at(0).psi.size(); i++)
    {
        final_feasible_armangle.push_back(std::make_pair(unsorted_feasible_psi.at(0).psi.at(i).first, unsorted_feasible_psi.at(0).psi.at(i).second));
    }

    if (size_unsorted_feasible_psi == 1)
        return succeeded;

    for(int i = 0; i < size_unsorted_feasible_psi-1; i++)
    {
        for(size_t first_range_num = 0; first_range_num < final_feasible_armangle.size(); first_range_num++)
        {
            for(size_t second_range_num = 0; second_range_num < unsorted_feasible_psi.at(i+1).psi.size(); second_range_num++)
            {
                case1_start = final_feasible_armangle.at(first_range_num).first;
                case1_end   = final_feasible_armangle.at(first_range_num).second;
                case2_start = unsorted_feasible_psi.at(i+1).psi.at(second_range_num).first;
                case2_end   = unsorted_feasible_psi.at(i+1).psi.at(second_range_num).second;

                /*
                        <------>
                    <-------------->
                    <------>
                    **************************
                        <------>
                            <->
                            <---------->
                    **************************

                */
                if( ((case2_start <= case1_start) && (case2_end >= case1_start) ) ||
                    ((case2_start >= case1_start) && (case2_start <= case1_end)))
                {

                    psi_pair.first= std::max(case1_start, case2_start);
                    psi_pair.second = std::min(case1_end, case2_end);

                    //psi_pair.first= std::max(result.at(first_range_num).first, unsorted_feasible_psi.at(i+1).psi.at(second_range_num).first);
                    //psi_pair.second = std::min(result.at(first_range_num).second, unsorted_feasible_psi.at(i+1).psi.at(second_range_num).second);

                    temp_result.push_back(psi_pair);
                }
            }
        }
        final_feasible_armangle.clear();
        for(size_t temp_result_num = 0; temp_result_num < temp_result.size(); temp_result_num++)
        {
            final_feasible_armangle.push_back(temp_result.at(temp_result_num));                        
        }
        temp_result.clear();
    }

    if(final_feasible_armangle.size()<1)
        return SRSKinematic::ERR_UNION_ALL;
    else
        return succeeded;

}

bool check_for_psi_range( const std::pair< double,double > psi_pair)
{
    if( ((psi_pair.first >= -kinematics_library::PI) && (psi_pair.first <= kinematics_library::PI)) &&
        ((psi_pair.second >= -kinematics_library::PI) && (psi_pair.second <= kinematics_library::PI)) )
            return true;
    else
        return false;
}
