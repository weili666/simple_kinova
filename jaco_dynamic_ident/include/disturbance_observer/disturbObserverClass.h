#ifndef __DISTURBANCEOBSERVERCLASS_H__
#define __DISTURBANCEOBSERVERCLASS_H__
#include <ros/ros.h>

//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

typedef Eigen::Matrix<double, 6, 1>       JointVector;
typedef Eigen::Matrix<double, 6, 6> GainMatrix;
class disturbObserverClass
{


public:
	disturbObserverClass();
	~disturbObserverClass();
             //Setter and getter for sample time and gain matrix
	void updateDO(JointVector &disturb_vector, JointVector &torque_command, JointVector &torque_invdyn);
	void setTsample(double t);
	void setGain(JointVector g);
	double getTsample() const;
	GainMatrix getGain() const;
            //Print all the parameter
	void printParam();

private:
	double time_sample;
	GainMatrix gain_matrix;

};

#endif
