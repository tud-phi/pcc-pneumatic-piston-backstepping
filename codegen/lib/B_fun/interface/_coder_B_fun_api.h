//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_B_fun_api.h
//
// Code generation for function 'B_fun'
//

#ifndef _CODER_B_FUN_API_H
#define _CODER_B_FUN_API_H

// Include files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void B_fun(real_T q[3], real_T alpha, real_T l[3], real_T rho[3], real_T B[9]);

void B_fun_api(const mxArray *const prhs[4], const mxArray **plhs);

void B_fun_atexit();

void B_fun_initialize();

void B_fun_terminate();

void B_fun_xil_shutdown();

void B_fun_xil_terminate();

#endif
// End of code generation (_coder_B_fun_api.h)
