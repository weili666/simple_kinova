#ifndef __NONLINEARDO_H__
#define __NONLINEARDO_H__
#include <ros/ros.h>

//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

//joint state msg related
#include <wpi_jaco_msgs/JointPosiCartForce.h>
#include <std_msgs/Float32MultiArray.h>

//Self defined class related
#include <disturbance_observer/disturbObserverClass.h>
#include <jaco_regressor/jacoRegressorClass.h>
#include <neural_network/nnCompensatorClass.h>
#include <torque2force/torque2ForceClass.h>



typedef Eigen::Matrix<double, 6, 1>       JointVector;

class nonLinearDO
{
public:
	nonLinearDO(ros::NodeHandle nh);

	~nonLinearDO();

    void NdoCallback(const wpi_jaco_msgs::JointPosiCartForce::ConstPtr& joint_cartesian_states);
private:
	JointVector disturb_vector;
    //std::vector<double> force_vector;
	ros::Subscriber nonDOSubscriber;
	ros::Publisher disturbancePublisher;
	
	jacoRegressorClass regressor;
	nnCompensatorClass compensator;
	disturbObserverClass observer;
	torque2ForceClass converter;


};

#endif
