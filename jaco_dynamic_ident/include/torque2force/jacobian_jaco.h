//
// File: jacobian_jaco.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Mar-2016 14:54:58
//
#ifndef __JACOBIAN_JACO_H__
#define __JACOBIAN_JACO_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "torque2force_types.h"

// Function Declarations
extern void jacobian_jaco(double D2, double D3, double D4, double D5, double D6,
  double e2, double q1, double q2, double q3, double q4, double q5, double q6,
  double jaco[36]);

#endif

//
// File trailer for jacobian_jaco.h
//
// [EOF]
//
