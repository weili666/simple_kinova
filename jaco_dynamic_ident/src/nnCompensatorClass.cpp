#include <neural_network/nnCompensatorClass.h>

nnCompensatorClass::nnCompensatorClass()
{
	neuralFcn_initialize();
}

nnCompensatorClass::~nnCompensatorClass()
{
	neuralFcn_terminate();
}

void nnCompensatorClass::genJointTorque(std::vector<double> posivel_vector,  JointVector &torque_vector)
{
	double posivel_array[12];
	double torque_array[6];

	for (unsigned int i = 0; i < 12; i++)
	{
		posivel_array[i] = posivel_vector[i];
	}


	neuralFcn(posivel_array, torque_array);
	for (unsigned int i = 0; i < 6; i++)
	{
		torque_vector(i) = torque_array[i];
	}
}