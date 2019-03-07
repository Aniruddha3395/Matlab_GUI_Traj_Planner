#include "mex.h"
#include "matrix.h"
#include "string.h"
#include <stdio.h>

#include "iiwa7_FK_mex.hpp"

void mexFunction (int _no_op_args, mxArray *FF__Base[], int _no_ip_args, const mxArray *b_ip[] )
{
    // I/O variables
    double* theta;
    double* base_T;
    mxDouble T_WE[16];

    // input
    theta = mxGetDoubles( b_ip[0] );
    base_T = mxGetDoubles( b_ip[1] );
    
    // process
    iiwa7_FK_mex( theta, base_T, T_WE );
    
    // output
    FF__Base[0] = mxCreateDoubleMatrix( 4, 4, mxREAL );    
    memcpy(mxGetPr(FF__Base[0]), T_WE, 4 * 4 * sizeof(double));
    
    return;
    
}