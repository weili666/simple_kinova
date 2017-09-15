#include <ros/ros.h>

//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

//NN compensator related
#include <neural_network/nnCompensatorClass.h>

//Used to read pre-stored trajectory file
//C++ file output/input
#include <iostream>
#include <fstream>

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int argc, char **argv)
{
	nnCompensatorClass nn_compensator;
	Eigen::Matrix<double, 6, 1>       joint_torque;
	std::vector<double> pv_vector;
	// pv_vector.resize(12);
	pv_vector.push_back(-1.66127);
	pv_vector.push_back(4.48659);
	pv_vector.push_back(1.39851);
	pv_vector.push_back(2.32882);

	pv_vector.push_back(1.98611);
	pv_vector.push_back(2.42521);
	pv_vector.push_back(0);
	pv_vector.push_back(0);

	pv_vector.push_back(0);
	pv_vector.push_back(0);
	pv_vector.push_back(0);
	pv_vector.push_back(0);

	nn_compensator.genJointTorque(pv_vector, joint_torque);
	for (unsigned int i = 0; i < 6; i++)
	{
	  std::cout << joint_torque[i] << std::endl;
	}
	return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//

