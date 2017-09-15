#include <ros/ros.h>

//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

//jaco regressor related
// #include <jaco_regressor/rt_nonfinite.h>
// #include <jaco_regressor/jaco_regressor.h>
// #include <jaco_regressor/jaco_regressor_terminate.h>
// #include <jaco_regressor/jaco_regressor_initialize.h>
#include <jaco_regressor/jacoRegressorClass.h>

//Used to read pre-stored trajectory file
//C++ file output/input
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
	jacoRegressorClass jaco_regre;
	Eigen::Matrix<double, 5, 12> regressor;
	//std::vector<double> regressorVector;
	jaco_regre.genRegressorMatrix(4.486587008407923, 1.4013967777546459, 2.3252549499122424, 1.9765853778835782, 2.420454436767085, regressor);
	JacoParamVector g_param;
	g_param << 11.178473,
	        2.6460264,
	        - 33.256607,
	        - 28.698364,
	        1.1949130,
	        1.2702314,
	        7.4337735,
	        10.865885,
	        - 11.437148,
	        - 4.6886101,
	        5.3724060,
	        0.22759987;
	JointVector torque_vector;
	torque_vector << 0, regressor * g_param; //5*1 vector, add 0 in the first place
	for (unsigned int i = 0; i < 6; i++)
	{
		std::cout << torque_vector(i) << std::endl;
	}
	std::cout << "gravity torque generated" << std::endl;


	return 0;

}
