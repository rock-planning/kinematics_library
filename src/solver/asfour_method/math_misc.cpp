#include "kinematics_library/solver/asfour_method/math_misc.hpp"
#include <stdio.h>
using namespace kinematics_library;
void kinematics_library::Eul2RotMat(double eul_zyx[3], double *rot_mat)
{
  double ca=cos(eul_zyx[0]), sa=sin(eul_zyx[0]), cb=cos(eul_zyx[1]), sb=sin(eul_zyx[1]), cg=cos(eul_zyx[2]), sg=sin(eul_zyx[2]);

  rot_mat[0]= ca*cb;   rot_mat[3]=(ca*sb*sg)-(sa*cg);     rot_mat[6]=(ca*sb*cg)+(sa*sg);
  rot_mat[1]= sa*cb;   rot_mat[4]=(sa*sb*sg)+(ca*cg);     rot_mat[7]=(sa*sb*cg)-(ca*sg);
  rot_mat[2]=-sb;      rot_mat[5]=cb*sg;                  rot_mat[8]=cb*cg;
}

void kinematics_library::Tra2Eul_pos(double tra[16], double *rot, double *pos)
{

  pos[0]=tra[12];
  pos[1]=tra[13];
  pos[2]=tra[14];

  rot[1]=atan2(-tra[2],sqrt((tra[0]*tra[0])+(tra[1]*tra[1])));
  rot[0]=atan2((tra[1]/(cos(rot[1]))),(tra[0]/cos(rot[1])));
  rot[2]=atan2((tra[6]/(cos(rot[1]))),(tra[10]/cos(rot[1])));


}

double kinematics_library::norm_vec(double vec[3]){

  double result = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
  return result;
}

void kinematics_library::scale_vec(double v[3], double factor, double *res){
  int i;
  for(i = 0; i < 3; i++){
    res[i] = v[i] * factor;
  }
}

void kinematics_library::add_vec(double v1[3], double v2[3], double *res){
  int i;
  for(i = 0; i < 3; i++){
    res[i] = v1[i] + v2[i];
  }
}

void kinematics_library::cross_vec(double v1[3], double v2[3], double *res){
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}


void kinematics_library::extract_rot_Mat(double mat[16], double *res){
  res[0] = mat[0];
  res[1] = mat[1];
  res[2] = mat[2];
  res[3] = mat[4];
  res[4] = mat[5];
  res[5] = mat[6];
  res[6] = mat[8];
  res[7] = mat[9];
  res[8] = mat[10];
}

void kinematics_library::build_transformation_mat(double rot_mat[9], double vec[3], double *res){
  res[0]=rot_mat[0];  res[4]=rot_mat[3];   res[8]=rot_mat[6];   res[12]=vec[0];
  res[1]=rot_mat[1];  res[5]=rot_mat[4];   res[9]=rot_mat[7];   res[13]=vec[1];
  res[2]=rot_mat[2];  res[6]=rot_mat[5];   res[10]=rot_mat[8];  res[14]=vec[2];
  res[3]=0.0;         res[7]=0.0;          res[11]=0.0;         res[15]= 1.0;
}

void kinematics_library::rotationMatrix2zyzEuler(double rotMat[9], double *zyzEuler){
  zyzEuler[1] = atan2(sqrt(rotMat[2]*rotMat[2] + rotMat[5]*rotMat[5]), rotMat[8]);

  if(zyzEuler[1] < 0.001){					//ca. 0 -> 0
    zyzEuler[0] = 0.0;
    zyzEuler[1] = 0.0;
    zyzEuler[2] = atan2(-rotMat[3], rotMat[0]);
  }
  else if(zyzEuler[1] > PI-0.001){			//ca. PI -> PI
    zyzEuler[0] = 0.0;
    zyzEuler[1] = PI;
    zyzEuler[2] = atan2(rotMat[3], -rotMat[0]);
  }
  else{
    zyzEuler[0] = atan2(rotMat[7]/sin(zyzEuler[1]), rotMat[6]/sin(zyzEuler[1]));
    zyzEuler[2] = atan2(rotMat[5]/sin(zyzEuler[1]), -rotMat[2]/sin(zyzEuler[1]));
  }
}

void kinematics_library::copy_mat_4x4(double src[16], double *dest){
  unsigned short j;
  for(j = 0; j < 16; j++)
    dest[j] = src[j];
}

void kinematics_library::Mult_mat_vec(double mat[9], double vec[3], double *res)
{
  res[0]=(mat[0]*vec[0])+(mat[3]*vec[1])+(mat[6]*vec[2]);
  res[1]=(mat[1]*vec[0])+(mat[4]*vec[1])+(mat[7]*vec[2]);
  res[2]=(mat[2]*vec[0])+(mat[5]*vec[1])+(mat[8]*vec[2]);
}

void kinematics_library::Mult_mat_mat_3x3(double mat1[9], double mat2[9], double *res)
{
  res[0]=(mat1[0]*mat2[0])+(mat1[3]*mat2[1])+(mat1[6]*mat2[2]);    res[3]=(mat1[0]*mat2[3])+(mat1[3]*mat2[4])+(mat1[6]*mat2[5]);    res[6]=(mat1[0]*mat2[6])+(mat1[3]*mat2[7])+(mat1[6]*mat2[8]);
  res[1]=(mat1[1]*mat2[0])+(mat1[4]*mat2[1])+(mat1[7]*mat2[2]);    res[4]=(mat1[1]*mat2[3])+(mat1[4]*mat2[4])+(mat1[7]*mat2[5]);    res[7]=(mat1[1]*mat2[6])+(mat1[4]*mat2[7])+(mat1[7]*mat2[8]);
  res[2]=(mat1[2]*mat2[0])+(mat1[5]*mat2[1])+(mat1[8]*mat2[2]);    res[5]=(mat1[2]*mat2[3])+(mat1[5]*mat2[4])+(mat1[8]*mat2[5]);    res[8]=(mat1[2]*mat2[6])+(mat1[5]*mat2[7])+(mat1[8]*mat2[8]);
}

void kinematics_library::mult_mat_mat_4x4(double src1[16], double src2[16], double *dest)
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

void kinematics_library::rot_matrix(double theta, double alpha, double *res)
{
  double ca=cos(alpha), sa=sin(alpha), ct=cos(theta), st=sin(theta);

  res[0]=ct;  res[3]=-st*ca;   res[6]=st*sa;
  res[1]=st;  res[4]=ct*ca;    res[7]=-ct*sa;
  res[2]=0;   res[5]=sa;       res[8]=ca;
}
void Mult_vec_tslvec(double vec[3], double *res)
{
  res[0]=vec[0]*vec[0];  res[3]=vec[0]*vec[1];   res[6]=vec[0]*vec[2];
  res[1]=vec[1]*vec[0];  res[4]=vec[1]*vec[1];   res[7]=vec[1]*vec[2];
  res[2]=vec[2]*vec[0];  res[5]=vec[2]*vec[1];   res[8]=vec[2]*vec[2];

}
void kinematics_library::trans_mat(double mat[9], double *res)
{
  res[0]=mat[0];  res[3]=mat[1];   res[6]=mat[2];
  res[1]=mat[3];  res[4]=mat[4];   res[7]=mat[5];
  res[2]=mat[6];  res[5]=mat[7];   res[8]=mat[8];
}

void kinematics_library::invert_mat(double mat[16], double *res)
{
  double rot_mat[9];
  double t_mat[9];
  double pos_vec[3] = {mat[12], mat[13], mat[14]};
  double new_pos[3];
  double new_pos_neg[3];

  kinematics_library::extract_rot_Mat(mat, rot_mat);
  kinematics_library::trans_mat(rot_mat, t_mat);
  kinematics_library::Mult_mat_vec(t_mat, pos_vec, new_pos);
  kinematics_library::scale_vec(new_pos, -1, new_pos_neg);

  res[0]=t_mat[0];  res[4]=t_mat[3];   res[8]=t_mat[6];   res[12]=new_pos_neg[0];
  res[1]=t_mat[1];  res[5]=t_mat[4];   res[9]=t_mat[7];   res[13]=new_pos_neg[1];
  res[2]=t_mat[2];  res[6]=t_mat[5];   res[10]=t_mat[8];  res[14]=new_pos_neg[2];
  res[3]=0.0;       res[7]=0.0;        res[11]=0.0;       res[15]=1.0;
}

void kinematics_library::identityMatrix(double *dest)
{
  unsigned short i;
  for(i = 0; i < 16; i++)
    dest[i] = 0.0;
  dest[0] = 1.0;
  dest[5] = 1.0;
  dest[10] = 1.0;
  dest[15] = 1.0;
}

double kinematics_library::double_modulo(double divident, double divisor){
  // double d_result = divident / divisor;
  // printf("divident %f / divisor %f = %f\n", divident, divisor, d_result);
  int int_result = (int) ( divident / divisor );
  // printf("int_result: %d\n", int_result);
  double result = (divident - (double) int_result * divisor);
  // printf("%f transforms to %f\n", divident, result);
  return result;
}

void kinematics_library::rotationMatrix2zyxEuler(double *rotMat, double * zyxEuler)
{
    zyxEuler[1] = atan2(-rotMat[2], sqrt(rotMat[0]*rotMat[0] + rotMat[1]*rotMat[1]));
    
    if(zyxEuler[1] < -M_PI/2.0+0.001){//ca. -PI/2
        zyxEuler[0] = 0.0;
        zyxEuler[1] = -M_PI/2.0;
        zyxEuler[2] = -atan2(rotMat[3], rotMat[4]);
    }
    else if(zyxEuler[1] > M_PI/2.0-0.001){//ca. PI/2
        zyxEuler[0] = 0.0;
        zyxEuler[1] = M_PI/2.0;
        zyxEuler[2] = atan2(rotMat[3], rotMat[4]);
    }
    else{
        zyxEuler[0] = atan2(rotMat[1]/cos(zyxEuler[1]), rotMat[0]/cos(zyxEuler[1]));
        zyxEuler[2] = atan2(rotMat[5]/cos(zyxEuler[1]), rotMat[8]/cos(zyxEuler[1]));
    }
}

void kinematics_library::homogeneousMatrix2zyxEuler_pos ( double* tra, double* zyxEuler, double* pos )
{
    pos[0]=tra[12];
    pos[1]=tra[13];
    pos[2]=tra[14];
    
    zyxEuler[1] = atan2(-tra[2], sqrt(tra[0]*tra[0] + tra[1]*tra[1]));
    
    if(zyxEuler[1] < -M_PI/2.0+0.001){//ca. -PI/2
        zyxEuler[0] = 0.0;
        zyxEuler[1] = -M_PI/2.0;
        zyxEuler[2] = -atan2(tra[4], tra[5]);
    }
    else if(zyxEuler[1] > M_PI/2.0-0.001){//ca. PI/2
        zyxEuler[0] = 0.0;
        zyxEuler[1] = M_PI/2.0;
        zyxEuler[2] = atan2(tra[4], tra[5]);
    }
    else{
        zyxEuler[0] = atan2(tra[1]/cos(zyxEuler[1]), tra[0]/cos(zyxEuler[1]));
        zyxEuler[2] = atan2(tra[6]/cos(zyxEuler[1]), tra[10]/cos(zyxEuler[1]));
    }
}

void kinematics_library::homogeneousMatrix2rotationMatrix_pos ( double* tra, double* rot_mat, double* pos )
{
    pos[0]=tra[12];
    pos[1]=tra[13];
    pos[2]=tra[14];
    
    rot_mat[0] = tra[0];    rot_mat[3] = tra[4];    rot_mat[6] = tra[8];
    rot_mat[1] = tra[1];    rot_mat[4] = tra[5];    rot_mat[7] = tra[9];
    rot_mat[2] = tra[2];    rot_mat[5] = tra[6];    rot_mat[8] = tra[10];
}

void kinematics_library::Array_2_EigenMat_4x4(double* src, Eigen::MatrixXd& dest)
{
    if(dest.rows() != 4 || dest.cols() != 4 )
    {
        dest.resize(4,4);
    }
    dest(0,0) = src[0];     dest(0,1) = src[4];     dest(0,2) = src[8];     dest(0,3) = src[12];
    dest(1,0) = src[1];     dest(1,1) = src[5];     dest(1,2) = src[9];     dest(1,3) = src[13];
    dest(2,0) = src[2];     dest(2,1) = src[6];     dest(2,2) = src[10];    dest(2,3) = src[14];
    dest(3,0) = src[3];     dest(3,1) = src[7];     dest(3,2) = src[11];    dest(3,3) = src[15];
}

void EigenMat4x4_2_Array(Eigen::MatrixXd& src, double* dest)
{
    dest[0] = src(0,0);     dest[4] = src(0,1);     dest[8] = src(0,2);     dest[12] = src(0,3);
    dest[1] = src(1,0);     dest[5] = src(1,1);     dest[9] = src(1,2);     dest[13] = src(1,3);
    dest[2] = src(2,0);     dest[6] = src(2,1);     dest[10] = src(2,2);    dest[14] = src(2,3);
    dest[3] = src(3,0);     dest[7] = src(3,1);     dest[11] = src(3,2);    dest[15] = src(3,3);
}

void kinematics_library::EigenMat3x3_2_Array(base::MatrixXd& src, double* dest)
{
    dest[0] = src(0,0);     dest[3] = src(0,1);     dest[6] = src(0,2);     
    dest[1] = src(1,0);     dest[4] = src(1,1);     dest[7] = src(1,2);    
    dest[2] = src(2,0);     dest[5] = src(2,1);     dest[8] = src(2,2); 
}

