#include "mex.h"
#include "matrix.h"
#include <iostream>
#include "string.h"
#include </usr/local/include/eigen3/Eigen/Eigen>
#include <stdio.h>
#include <vector>
#include <cmath>

Eigen::MatrixXd bxbybz2eul(Eigen::MatrixXd);
Eigen::MatrixXd rot2eul(Eigen::Matrix3d, std::string);
std::string validate_seq(std::string);

void mexFunction (int _OutArgs, mxArray *MatlabOut[], int _InArgs, const mxArray *MatlabIn[] )
{
    // Define Input
    int bx_cols = 3;
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> bx (mxGetPr(MatlabIn[0]), mxGetNumberOfElements(MatlabIn[0])/bx_cols, bx_cols); 
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> by (mxGetPr(MatlabIn[1]), mxGetNumberOfElements(MatlabIn[1])/bx_cols, bx_cols); 
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> bz (mxGetPr(MatlabIn[2]), mxGetNumberOfElements(MatlabIn[2])/bx_cols, bx_cols); 
    
    Eigen::MatrixXd bxbybz(bx.rows(),9);
    bxbybz.block(0,0,bxbybz.rows(),bxbybz.cols()) << bx;
    bxbybz.block(0,3,bxbybz.rows(),bxbybz.cols()) << by;
    bxbybz.block(0,6,bxbybz.rows(),bxbybz.cols()) << bz;
     
    // Method 
    Eigen::MatrixXd cba = bxbybz2eul(bxbybz);
    
    // Define Output
    MatlabOut[0] = mxCreateDoubleMatrix(cba.rows(), cba.cols(), mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M0 ( mxGetPr(MatlabOut[0]), cba.rows(), cba.cols() );
    M0 = cba.array();  
}

Eigen::MatrixXd bxbybz2eul(Eigen::MatrixXd bxbybz)
{
	Eigen::MatrixXd Gx(1,3), Gy(1,3), Gz(1,3);
	Gx << 1,0,0;
	Gy << 0,1,0;
	Gz << 0,0,1;
	Eigen::Matrix3d R = Eigen::Matrix3d::Constant(0);
	Eigen::MatrixXd cba = Eigen::MatrixXd::Constant(bxbybz.rows(), 3, 0);
	for (unsigned int i=0;i<bxbybz.rows();++i)
	{
		R(0,0) = bxbybz(i,0)*Gx(0,0)+bxbybz(i,1)*Gx(0,1)+bxbybz(i,2)*Gx(0,2);
		R(0,1) = bxbybz(i,3)*Gx(0,0)+bxbybz(i,4)*Gx(0,1)+bxbybz(i,5)*Gx(0,2);
		R(0,2) = bxbybz(i,6)*Gx(0,0)+bxbybz(i,7)*Gx(0,1)+bxbybz(i,8)*Gx(0,2);
		R(1,0) = bxbybz(i,0)*Gy(0,0)+bxbybz(i,1)*Gy(0,1)+bxbybz(i,2)*Gy(0,2);
		R(1,1) = bxbybz(i,3)*Gy(0,0)+bxbybz(i,4)*Gy(0,1)+bxbybz(i,5)*Gy(0,2);
		R(1,2) = bxbybz(i,6)*Gy(0,0)+bxbybz(i,7)*Gy(0,1)+bxbybz(i,8)*Gy(0,2);
		R(2,0) = bxbybz(i,0)*Gz(0,0)+bxbybz(i,1)*Gz(0,1)+bxbybz(i,2)*Gz(0,2);
		R(2,1) = bxbybz(i,3)*Gz(0,0)+bxbybz(i,4)*Gz(0,1)+bxbybz(i,5)*Gz(0,2);
		R(2,2) = bxbybz(i,6)*Gz(0,0)+bxbybz(i,7)*Gz(0,1)+bxbybz(i,8)*Gz(0,2);
		cba.row(i) = rot2eul(R,"ZYX");	
	}
	return cba;	
}

Eigen::MatrixXd rot2eul(Eigen::Matrix3d rot_mat, std::string seq)
{
	seq = validate_seq(seq);
	int rot_idx[3];
	for (int i=0; i<3; ++i)
	{
		if(seq[i]=='X')
			rot_idx[i] = 0;
		else if(seq[i]=='Y')
			rot_idx[i] = 1;
		else if(seq[i]=='Z')
			rot_idx[i] = 2;
	}	
	Eigen::MatrixXd eul_angles(1,3);
	Eigen::Vector3d eul_angles_vec;
	eul_angles_vec = rot_mat.eulerAngles(rot_idx[0], rot_idx[1], rot_idx[2]);
	eul_angles(0,0) = eul_angles_vec[0];
	eul_angles(0,1) = eul_angles_vec[1];
	eul_angles(0,2) = eul_angles_vec[2];
	return eul_angles;
}

std::string validate_seq(std::string seq)
{
	if(seq =="")
		seq = "ZYX";	
	bool invalid_flag = false;
	if(seq.size()>0 && seq.size()<3)
	{
		invalid_flag = true;
	}
	for (int i =0;i<3;++i)
		if(seq[i]!='X' && seq[i]!='Y' && seq[i]!='Z')
		{
			invalid_flag = true; 
			break;
		}
	if(invalid_flag)
	{
		std::cerr << "ERROR: Invalid Rotations Sequence: " << seq << std::endl;
		std::terminate();		
	}
	return seq;
}