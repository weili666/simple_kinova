//
// File: torque2force.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Mar-2016 14:54:58
//
#ifndef __TORQUE2FORCE_H__
#define __TORQUE2FORCE_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "torque2force_types.h"

// Function Declarations
extern void torque2force(const double joint_torque[6], const double
  joint_position[6], double cartesian_force[6]);

#endif

//
// File trailer for torque2force.h
//
// [EOF]
//
