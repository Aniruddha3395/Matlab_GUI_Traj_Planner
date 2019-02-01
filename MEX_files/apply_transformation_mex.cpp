#include "mex.h"
#include "matrix.h"
#include <iostream>
#include "string.h"
#include </usr/local/include/eigen3/Eigen/Eigen>
#include <stdio.h>
#include <vector>
#include <cmath>

Eigen::MatrixXd apply_transformation(Eigen::MatrixXd, Eigen::Matrix4d);

void mexFunction (int _OutArgs, mxArray *MatlabOut[], int _InArgs, const mxArray *MatlabIn[] )
{
    // Define Input
    int data_cols = 3;
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> data (mxGetPr(MatlabIn[0]), mxGetNumberOfElements(MatlabIn[0])/data_cols, data_cols); 
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> T_mat (mxGetPr(MatlabIn[1]), 4, 4); 
     
    // Method 
    Eigen::MatrixXd transformed_data = apply_transformation(data, T_mat);
    
    // Define Output
    MatlabOut[0] = mxCreateDoubleMatrix(transformed_data.rows(), transformed_data.cols(), mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M0 ( mxGetPr(MatlabOut[0]), transformed_data.rows(), transformed_data.cols() );
    M0 = transformed_data.array();  
}

Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
{
    //NOTE: Homogeneous Tranformation Matrix (4x4)

    // putting data in [x, y, z, 1]' format
    Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
    Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
    data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
    data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
    Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
    Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
    transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
    return transformed_data_mat.transpose();
}