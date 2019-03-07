#include "mex.h"
#include "matrix.h"
#include "string.h"
#include <stdio.h>
#include <cmath>
#include </usr/local/include/eigen3/Eigen/Eigen>
#include "nlopt.hpp"
#include <iostream>
#include <vector>
#include "opt_obj.hpp"
#include "iiwa_utilities.hpp"
#include <chrono>

void mexFunction (int _no_op_args, mxArray *mex_op[], int _no_ip_args, const mxArray *mex_in[] )
{
    /////////////////// DECLARATIONS ///////////////////////////////////
    // I/O variables

    /////////////////// INPUT ///////////////////////////////////
    double *ptr_rob_version;
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> joint_config (mxGetPr(mex_in[0]),7,1);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> xyz_bxbybz_T (mxGetPr(mex_in[1]),1,12);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> robot_ree_T_tee (mxGetPr(mex_in[2]),4,4);
    ptr_rob_version = mxGetDoubles(mex_in[3]);
    int rob_version = (int) *ptr_rob_version;

    /////////////////// CREATING OPTIMIZATION OBJECT //////////////////

    double OptH = 1e-8;
    double OptXtolRel = 1e-10;
    
    // std::vector<double> X_init{0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463,0};
    std::vector<double> X_init{joint_config(0,0),joint_config(1,0),joint_config(2,0),joint_config(3,0),joint_config(4,0),joint_config(5,0),joint_config(6,0)};

    opt_obj::opt_obj OptObj(joint_config, xyz_bxbybz_T, robot_ree_T_tee, X_init, OptH, OptXtolRel, rob_version);
    Eigen::MatrixXd M = OptObj.ascent_IK();

    /////////////////// OUTPUT ///////////////////////////////////
    mex_op[0] = mxCreateDoubleMatrix(7,1, mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M0 (mxGetPr(mex_op[0]),7,1);
    M0 = M.block(0,0,7,1);
    mex_op[1] = mxCreateDoubleMatrix(1,1, mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M1 (mxGetPr(mex_op[1]),1,1);
    M1 << M(7,0);  
    return;
}
