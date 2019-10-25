#include "kinematics_library/solver/shimizu_method/SRSKinematicHelper.hpp"

void Eul2RotMat(const double eul_zyx[3], std::vector<double> &rot_mat)
{
    double ca=cos(eul_zyx[0]), sa=sin(eul_zyx[0]), cb=cos(eul_zyx[1]), sb=sin(eul_zyx[1]), cg=cos(eul_zyx[2]), sg=sin(eul_zyx[2]);

    rot_mat.at(0)= ca*cb;   rot_mat.at(3)=(ca*sb*sg)-(sa*cg);     rot_mat.at(6)=(ca*sb*cg)+(sa*sg);
    rot_mat.at(1)= sa*cb;   rot_mat.at(4)=(sa*sb*sg)+(ca*cg);     rot_mat.at(7)=(sa*sb*cg)-(ca*sg);
    rot_mat.at(2)=-sb;      rot_mat.at(5)=cb*sg;                  rot_mat.at(8)=cb*cg;
}

void quaternionToRotMat(const Eigen::Quaternion<double> &quat, std::vector<double> &rot_mat)
{
//     rot_mat.at(0) = 1-2*((quat.y()*quat.y()) +(quat.z()*quat.z()));  rot_mat.at(3) = 2*((quat.x()*quat.y()) - (quat.z()*quat.w()));   rot_mat.at(6) = 2*((quat.x()*quat.z()) + (quat.y()*quat.w()));
//     rot_mat.at(1) = 2*((quat.x()*quat.y()) + (quat.z()*quat.w()));   rot_mat.at(4) = 1-2*((quat.x()*quat.x()) +(quat.z()*quat.z()));  rot_mat.at(7) = 2*((quat.y()*quat.z()) - (quat.x()*quat.w()));
//     rot_mat.at(2) = 2*((quat.x()*quat.z()) - (quat.y()*quat.w()));   rot_mat.at(5) = 2*((quat.y()*quat.z()) + (quat.x()*quat.w()));   rot_mat.at(8) =  1-2*((quat.x()*quat.x()) +(quat.y()*quat.y()));

    //use of eigen library
//     Eigen::Matrix3d quat_rot_mat = quat.normalized().toRotationMatrix();
//     for(int i = 0; i < 3; i++)
//     {
//         for(int j = 0; j < 3; j++)
//             rot_mat.at(i+j) = quat_rot_mat(j,i);
//     }
    
    Eigen::Matrix3d mat = quat.toRotationMatrix();
    rot_mat.at(0) = mat(0,0); rot_mat.at(3) = mat(0,1); rot_mat.at(6) = mat(0,2);
    rot_mat.at(1) = mat(1,0); rot_mat.at(4) = mat(1,1); rot_mat.at(7) = mat(1,2);
    rot_mat.at(2) = mat(2,0); rot_mat.at(5) = mat(2,1); rot_mat.at(8) = mat(2,2);
    
}

void Tra2Eul_pos(double tra[16], double *rot, double *pos)
{

    pos[0]=tra[12];
    pos[1]=tra[13];
    pos[2]=tra[14];

    rot[1]=atan2(-tra[2],sqrt((tra[0]*tra[0])+(tra[1]*tra[1])));
    rot[0]=atan2((tra[1]/(cos(rot[1]))),(tra[0]/cos(rot[1])));
    rot[2]=atan2((tra[6]/(cos(rot[1]))),(tra[10]/cos(rot[1])));
}

void Mult_mat_mat(const std::vector<double> &mat1, const std::vector<double> & mat2, std::vector<double> &res)
{
    res.at(0)=(mat1.at(0)*mat2.at(0))+(mat1.at(3)*mat2.at(1))+(mat1.at(6)*mat2.at(2));    res.at(3)=(mat1.at(0)*mat2.at(3))+(mat1.at(3)*mat2.at(4))+(mat1.at(6)*mat2.at(5));    res.at(6)=(mat1.at(0)*mat2.at(6))+(mat1.at(3)*mat2.at(7))+(mat1.at(6)*mat2.at(8));
    res.at(1)=(mat1.at(1)*mat2.at(0))+(mat1.at(4)*mat2.at(1))+(mat1.at(7)*mat2.at(2));    res.at(4)=(mat1.at(1)*mat2.at(3))+(mat1.at(4)*mat2.at(4))+(mat1.at(7)*mat2.at(5));    res.at(7)=(mat1.at(1)*mat2.at(6))+(mat1.at(4)*mat2.at(7))+(mat1.at(7)*mat2.at(8));
    res.at(2)=(mat1.at(2)*mat2.at(0))+(mat1.at(5)*mat2.at(1))+(mat1.at(8)*mat2.at(2));    res.at(5)=(mat1.at(2)*mat2.at(3))+(mat1.at(5)*mat2.at(4))+(mat1.at(8)*mat2.at(5));    res.at(8)=(mat1.at(2)*mat2.at(6))+(mat1.at(5)*mat2.at(7))+(mat1.at(8)*mat2.at(8));


}

void Mult_mat_vec(const std::vector<double> &mat, const std::vector<double> &vec, std::vector<double> &res)
{
    res.at(0)=(mat.at(0)*vec.at(0))+(mat.at(3)*vec.at(1))+(mat.at(6)*vec.at(2));
    res.at(1)=(mat.at(1)*vec.at(0))+(mat.at(4)*vec.at(1))+(mat.at(7)*vec.at(2));
    res.at(2)=(mat.at(2)*vec.at(0))+(mat.at(5)*vec.at(1))+(mat.at(8)*vec.at(2));
}

void rot_matrix(const double &theta, const double &alpha, std::vector<double> &res)
{
    double ca=cos(alpha), sa=sin(alpha), ct=cos(theta), st=sin(theta);

    if (fabs(alpha) == SRSKinematic::PI/2.0)
            ca = 0.0;

    for(int i = 0; i < 9; i++)
            res.at(i) = 0.0;

    res.at(0) = ct;  res.at(3) = -st*ca;    res.at(6) =  st*sa;
    res.at(1) = st;  res.at(4) =  ct*ca;    res.at(7) = -ct*sa;
    res.at(2) = 0.0;   res.at(5) =  sa;       res.at(8) =  ca;

    /*res.at(0) = ct;  res.at(1) = st*ca;    res.at(2) =  st*sa;
    res.at(3) = -st;  res.at(4) =  ct*ca;    res.at(5) = ct*sa;
    res.at(6) = 0.0;   res.at(7) =  -sa;       res.at(8) =  ca;*/
}

void Mult_vec_tslvec(const std::vector<double> &vec, std::vector<double> &res)
{
    res.at(0)=vec.at(0)*vec.at(0);  res.at(3)=vec.at(0)*vec.at(1);   res.at(6)=vec.at(0)*vec.at(2);
    res.at(1)=vec.at(1)*vec.at(0);  res.at(4)=vec.at(1)*vec.at(1);   res.at(7)=vec.at(1)*vec.at(2);
    res.at(2)=vec.at(2)*vec.at(0);  res.at(5)=vec.at(2)*vec.at(1);   res.at(8)=vec.at(2)*vec.at(2);
}

void trans_mat(const std::vector<double> &mat, std::vector<double> &res)
{
    res.at(0)=mat.at(0);  res.at(3)=mat.at(1);   res.at(6)=mat.at(2);
    res.at(1)=mat.at(3);  res.at(4)=mat.at(4);   res.at(7)=mat.at(5);
    res.at(2)=mat.at(6);  res.at(5)=mat.at(7);   res.at(8)=mat.at(8);
}

void identityMatrix(std::vector<double> &dest)
{
	for(int i = 0; i < 16; i++)
                dest.at(i) = 0.0;

        dest.at(0) = 1.0;
        dest.at(5) = 1.0;
        dest.at(10) = 1.0;
        dest.at(15) = 1.0;
}

void multMatrix(const std::vector<double> &src1, const std::vector<double> &src2, std::vector<double> &dest)
{
    dest.at(0)  = src1.at(0)*src2.at(0) + src1.at(4)*src2.at(1) + src1.at(8)*src2.at(2) + src1[12]*src2.at(3);
    dest.at(1)  = src1.at(1)*src2.at(0) + src1.at(5)*src2.at(1) + src1.at(9)*src2.at(2) + src1[13]*src2.at(3);
    dest.at(2)  = src1.at(2)*src2.at(0) + src1.at(6)*src2.at(1) + src1.at(10)*src2.at(2) + src1[14]*src2.at(3);
    dest.at(3)  = src1.at(3)*src2.at(0) + src1.at(7)*src2.at(1) + src1.at(11)*src2.at(2) + src1[15]*src2.at(3);
    dest.at(4)  = src1.at(0)*src2.at(4) + src1.at(4)*src2.at(5) + src1.at(8)*src2.at(6) + src1[12]*src2.at(7);
    dest.at(5)  = src1.at(1)*src2.at(4) + src1.at(5)*src2.at(5) + src1.at(9)*src2.at(6) + src1[13]*src2.at(7);
    dest.at(6)  = src1.at(2)*src2.at(4) + src1.at(6)*src2.at(5) + src1.at(10)*src2.at(6) + src1[14]*src2.at(7);
    dest.at(7)  = src1.at(3)*src2.at(4) + src1.at(7)*src2.at(5) + src1.at(11)*src2.at(6) + src1[15]*src2.at(7);
    dest.at(8)  = src1.at(0)*src2.at(8) + src1.at(4)*src2.at(9) + src1.at(8)*src2.at(10) + src1[12]*src2.at(11);
    dest.at(9)  = src1.at(1)*src2.at(8) + src1.at(5)*src2.at(9) + src1.at(9)*src2.at(10) + src1[13]*src2.at(11);
    dest.at(10) = src1.at(2)*src2.at(8) + src1.at(6)*src2.at(9) + src1.at(10)*src2.at(10) + src1[14]*src2.at(11);
    dest.at(11) = src1.at(3)*src2.at(8) + src1.at(7)*src2.at(9) + src1.at(11)*src2.at(10) + src1[15]*src2.at(11);
    dest[12] = src1.at(0)*src2[12] + src1.at(4)*src2[13] + src1.at(8)*src2[14] + src1[12]*src2[15];
    dest[13] = src1.at(1)*src2[12] + src1.at(5)*src2[13] + src1.at(9)*src2[14] + src1[13]*src2[15];
    dest[14] = src1.at(2)*src2[12] + src1.at(6)*src2[13] + src1.at(10)*src2[14] + src1[14]*src2[15];
    dest[15] = src1.at(3)*src2[12] + src1.at(7)*src2[13] + src1.at(11)*src2[14] + src1[15]*src2[15];
}

void multMatrix(double src1[16], double src2[16], double *dest)
{
    dest[0]  = src1[0]*src2[0] + src1[4]*src2[1] + src1[8]*src2[2] + src1[12]*src2[3];
    dest[1]  = src1[1]*src2[0] + src1[5]*src2[1] + src1[9]*src2[2] + src1[13]*src2[3];
    dest[2]  = src1[2]*src2[0] + src1[6]*src2[1] + src1[10]*src2[2] + src1[14]*src2[3];
    dest[3]  = src1[3]*src2[0] + src1[7]*src2[1] + src1[11]*src2[2] + src1[15]*src2[3];
    dest[4]  = src1[0]*src2[4] + src1[4]*src2[5] + src1[8]*src2[6] + src1[12]*src2[7];
    dest[5]  = src1[1]*src2[4] + src1[5]*src2[5] + src1[9]*src2[6] + src1[13]*src2[7];
    dest[6]  = src1[2]*src2[4] + src1[6]*src2[5] + src1[10]*src2[6] + src1[14]*src2[7];
    dest[7]  = src1[3]*src2[4] + src1[7]*src2[5] + src1[11]*src2[6] + src1[15]*src2[7];
    dest[8]  = src1[0]*src2[8] + src1[4]*src2[9] + src1[8]*src2[10] + src1[12]*src2[11];
    dest[9]  = src1[1]*src2[8] + src1[5]*src2[9] + src1[9]*src2[10] + src1[13]*src2[11];
    dest[10] = src1[2]*src2[8] + src1[6]*src2[9] + src1[10]*src2[10] + src1[14]*src2[11];
    dest[11] = src1[3]*src2[8] + src1[7]*src2[9] + src1[11]*src2[10] + src1[15]*src2[11];
    dest[12] = src1[0]*src2[12] + src1[4]*src2[13] + src1[8]*src2[14] + src1[12]*src2[15];
    dest[13] = src1[1]*src2[12] + src1[5]*src2[13] + src1[9]*src2[14] + src1[13]*src2[15];
    dest[14] = src1[2]*src2[12] + src1[6]*src2[13] + src1[10]*src2[14] + src1[14]*src2[15];
    dest[15] = src1[3]*src2[12] + src1[7]*src2[13] + src1[11]*src2[14] + src1[15]*src2[15];
}

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

        if((infeasible_psi.at(i).psi.at(0).first >= -SRSKinematic::PI) && (infeasible_psi.at(i).psi.at(0).second <= SRSKinematic::PI))
        {                       
            new_range.psi.at(0) = std::make_pair(-SRSKinematic::PI,(infeasible_psi.at(i).psi.at(0).first- SRSKinematic::HALFDEGREE )); // this HALFDEGREE is explained in document
            new_range.psi.at(1) = std::make_pair((infeasible_psi.at(i).psi.at(0).second + SRSKinematic::HALFDEGREE), SRSKinematic::PI); // this HALFDEGREE is explained in document

            complimented_infeasbile_psi.push_back(new_range);

        }
        else if( ((infeasible_psi.at(i).psi.at(0).first >= -SRSKinematic::PI) && (infeasible_psi.at(i).psi.at(0).first <= SRSKinematic::PI)) &&
                 ((infeasible_psi.at(i).psi.at(0).second >= -SRSKinematic::PI) && (infeasible_psi.at(i).psi.at(0).second >= SRSKinematic::PI)) )
        {
            new_range.psi.at(0) = std::make_pair(-SRSKinematic::PI,(infeasible_psi.at(i).psi.at(0).first- SRSKinematic::HALFDEGREE )); // this HALFDEGREE is explained in document
            new_range.psi.at(1) = std::make_pair(SRSKinematic::PI, SRSKinematic::PI);

            complimented_infeasbile_psi.push_back(new_range);

        }
        else if( ((infeasible_psi.at(i).psi.at(0).first <= -SRSKinematic::PI) && (infeasible_psi.at(i).psi.at(0).first <= SRSKinematic::PI)) &&
                 ((infeasible_psi.at(i).psi.at(0).second >= -SRSKinematic::PI) && (infeasible_psi.at(i).psi.at(0).second <= SRSKinematic::PI)) )
        {
            new_range.psi.at(0) = std::make_pair(-SRSKinematic::PI, -SRSKinematic::PI);
            new_range.psi.at(1) = std::make_pair((infeasible_psi.at(i).psi.at(0).second + SRSKinematic::HALFDEGREE), SRSKinematic::PI); // this HALFDEGREE is explained in document

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

    for(int i = 0; i < unsorted_feasible_psi.at(0).psi.size(); i++)
    {
        final_feasible_armangle.push_back(std::make_pair(unsorted_feasible_psi.at(0).psi.at(i).first, unsorted_feasible_psi.at(0).psi.at(i).second));
    }

    if (size_unsorted_feasible_psi == 1)
        return succeeded;

    for(int i = 0; i < size_unsorted_feasible_psi-1; i++)
    {
        for(int first_range_num = 0; first_range_num < final_feasible_armangle.size(); first_range_num++)
        {
            for(int second_range_num = 0; second_range_num < unsorted_feasible_psi.at(i+1).psi.size(); second_range_num++)
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
        for(int temp_result_num = 0; temp_result_num < temp_result.size(); temp_result_num++)
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
    if( ((psi_pair.first >= -SRSKinematic::PI) && (psi_pair.first <= SRSKinematic::PI)) &&
        ((psi_pair.second >= -SRSKinematic::PI) && (psi_pair.second <= SRSKinematic::PI)) )
            return true;
    else
        return false;
}
