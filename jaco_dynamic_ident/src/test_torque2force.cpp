//
// File: main.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Mar-2016 14:54:58
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include <torque2force/torque2ForceClass.h>
#include <iostream>

// Function Declarations
static void argInit_6x1_real_T(double result[6]);
static double argInit_real_T();
static void main_torque2force();

// Function Definitions

//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_6x1_real_T(double result[6])
{
  int b_j0;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 6; b_j0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 1.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_torque2force()
{
  double dv1[6];
  double dv2[6];
  double cartesian_force[6];

  // Initialize function 'torque2force' input arguments.
  // Initialize function input argument 'joint_torque'.
  // Initialize function input argument 'joint_position'.
  // Call the entry-point 'torque2force'.
  argInit_6x1_real_T(dv1);
  argInit_6x1_real_T(dv2);
  torque2force(dv1, dv2, cartesian_force);
  for (unsigned int i = 0; i < 6; i++)
  {
    std::cout << cartesian_force[i] << std::endl;
  }
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
//  // Initialize the application.
//  // You do not need to do this more than one time.
//  torque2force_initialize();

//  // Invoke the entry-point functions.
//  // You can call entry-point functions multiple times.
//  main_torque2force();

//  // Terminate the application.
//  // You do not need to do this more than one time.
//  torque2force_terminate();
    torque2ForceClass converter;
    std::vector<double> posi, torque,force;
    for(int i = 0;i<6;i++)
        {
posi.push_back(1.0);
torque.push_back(1.0);
    }
    converter.genCatersianForce(posi,torque,force);
    for(int i = 0;i<6;i++)
        {
    std::cout << force[i] << std::endl;
    }
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
