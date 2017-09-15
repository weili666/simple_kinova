#ifndef __TORQUE2FORCECLASS_H__
#define __TORQUE2FORCECLASS_H__
#include <ros/ros.h>

#include <torque2force/rt_nonfinite.h>
#include <torque2force/torque2force.h>
#include <torque2force/torque2force_terminate.h>
#include <torque2force/torque2force_initialize.h>


class torque2ForceClass
{
public:
	torque2ForceClass();
	~torque2ForceClass();

	void genCatersianForce(const std::vector<double> &posi_vector, const std::vector<double> &torque_vector,
	                       std::vector<double> &force_vector );

};


#endif
