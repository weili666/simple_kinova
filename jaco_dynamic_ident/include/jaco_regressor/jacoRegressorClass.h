#ifndef __JACOREGRESSORCLASS_H__
#define __JACOREGRESSORCLASS_H__
#include <ros/ros.h>

//jaco regressor related
#include <jaco_regressor/rt_nonfinite.h>
#include <jaco_regressor/jaco_regressor.h>
#include <jaco_regressor/jaco_regressor_terminate.h>
#include <jaco_regressor/jaco_regressor_initialize.h>

//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

typedef Eigen::Matrix<double, 5, 12>       JacoRegressorMatrix;
typedef Eigen::Matrix<double, 12, 1>       JacoParamVector;
typedef Eigen::Matrix<double, 6, 1>       JointVector;

class jacoRegressorClass
{
public:
	jacoRegressorClass();
	~jacoRegressorClass();
	// JacoRegressorMatrix regressor;
	// std::vector<double> regressorVector;
	//Start from q2, because first joint value has no effect on the final gravity torque 
    void genRegressorMatrix(double q2, double q3, double q4, double q5, double q6,  JacoRegressorMatrix &jacoMatrix);
    void genJointTorque(std::vector<double> posi_vector,  JointVector &torque_vector);
    void printParam();
    void setParam(JacoParamVector p);
    JacoParamVector getParam() const;
private:
    JacoParamVector gravity_param;


};

#endif

