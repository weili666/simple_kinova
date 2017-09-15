#include <torque2force/torque2ForceClass.h>

torque2ForceClass::torque2ForceClass()
{
	torque2force_initialize();
}

torque2ForceClass::~torque2ForceClass()
{
	torque2force_terminate();
}

void torque2ForceClass::genCatersianForce(const std::vector<double> &posi_vector, const std::vector<double> &torque_vector,
        std::vector<double> &force_vector )
{
	double posi[6];
	double torque[6];
	double cartesian_force[6];
	for (unsigned int i = 0; i < 6; i++)
	{
		posi[i] = posi_vector[i];
		torque[i] = torque_vector[i];
    }
    torque2force(torque, posi, cartesian_force);
	for (unsigned int i = 0; i < 6; i++)
	{
		force_vector.push_back(cartesian_force[i]);
	}
}
