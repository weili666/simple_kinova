#include <disturbance_observer/disturbObserverClass.h>

disturbObserverClass::disturbObserverClass()
{
	//Init sample time as 0.1s
	time_sample = 0.01;
	//Init gain matrix as identity matrix
	gain_matrix.setIdentity();
	//printParam();
}

disturbObserverClass::~disturbObserverClass()
{
}

void disturbObserverClass::updateDO(JointVector &disturb_vector, JointVector &torque_command, JointVector &torque_invdyn)
{
	JointVector temp_vector;
	JointVector error_vector = torque_invdyn - torque_command;
	temp_vector =  time_sample * (-1 * gain_matrix * disturb_vector + gain_matrix * (error_vector)) + disturb_vector;
	disturb_vector = temp_vector;
}

void disturbObserverClass::setTsample(double t)
{
	time_sample = t;
	//printParam();
}

double disturbObserverClass::getTsample() const
{
	return time_sample;
}

void disturbObserverClass::setGain(JointVector g)
{
	gain_matrix = g.asDiagonal();
	//printParam();
}

GainMatrix disturbObserverClass::getGain() const
{

	return gain_matrix;
}

void disturbObserverClass::printParam()
{
	//print sample time
	std::cout << "Sample time: " << time_sample << std::endl;
	//print Gain vector
	std::cout << "Gain vector of disturbance observer: " << std::endl;
	JointVector gain_vector = gain_matrix.diagonal();
	for (unsigned int i = 0; i < 6; i++)
	{
		std::cout << gain_vector(i) << " | ";
	}
	std::cout << std::endl;

}
