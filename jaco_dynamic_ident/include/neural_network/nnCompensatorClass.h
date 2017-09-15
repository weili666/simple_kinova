#ifndef __NNCOMPENSATORCLASS_H__
#define __NNCOMPENSATORCLASS_H__
#include <ros/ros.h>

//Neural network related
#include <neural_network/rt_nonfinite.h>
#include <neural_network/neuralFcn.h>
#include <neural_network/neuralFcn_terminate.h>
#include <neural_network/neuralFcn_initialize.h>

//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

class nnCompensatorClass
{
	typedef Eigen::Matrix<double, 6, 1>       JointVector;
public:
	nnCompensatorClass();
	~nnCompensatorClass();

	void genJointTorque(std::vector<double> posivel_vector,  JointVector &torque_vector);
};



#endif
