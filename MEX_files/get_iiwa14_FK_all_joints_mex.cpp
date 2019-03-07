#include "iiwa14_FK_all_joints_mex.hpp"

#include "mex.h"
#include "matrix.h"
#include "string.h"
#include <stdio.h>

void mexFunction (int _no_op_args, mxArray *FF__Base[], int _no_ip_args, const mxArray *b_ip[] )
{
    // I/O variables
    double* theta;
    double* base_T;
    mxDouble T_WE[144];

    // input
    theta = mxGetDoubles( b_ip[0] );
    base_T =mxGetDoubles( b_ip[1] );
    
    // process
    iiwa14_FK_all_joints_mex( theta, base_T, T_WE );
    
    // output
    FF__Base[0] = mxCreateDoubleMatrix( 36, 4, mxREAL );    
    memcpy(mxGetPr(FF__Base[0]), T_WE, 36 * 4 * sizeof(double));
    
    return;
    
}