#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <canlib.h>
#include <signal.h>
#include "MotorController.h"
#include <ros/console.h>
#include <unistd.h>

MotorController motorController;

void stop(int signo)
{
    motorController.setLaserMotorVelocity(0);
    ROS_WARN("Node exit! Motor stop!");
    _exit(0);
}
int main(int argc,char** argv){

    ros::init(argc,argv,"laer_motor_node",ros::init_options::NoSigintHandler);
	ROS_INFO("201");
    unsigned int time = 50000;

    //捕获各种信号，停止电机转动
    signal(SIGINT, stop);
    signal(SIGSTOP,stop);
    signal(SIGTSTP,stop);
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<sensor_msgs::JointState> ("/sensor/encoder", 1);
    motorController.initialize(1000000);
    if(! motorController.isSucceed() ){
            ROS_ERROR("motor initialize fail\n");
            return -1;
    }

    //TODO 使用amp的SetCountsPerUnit
    //电机齿轮有20个齿，减速器上的电机有30个齿，减速器的减速比为1：100，电机转一圈为4000counts。因此4000×3/2×100=600000counts为激光旋转1圈360°，激光的1°为1666counts
    const double scale=1666.667;
    double laserMotorUpperBound=9.*scale,laserMotorLowerBound=-44*scale,laserMotorSpeed=20*scale,laserMotorThreshold=5.*scale;
    double pos;
    motorController.getLaserMotorPosition(pos);

    sensor_msgs::JointState posMsg;
    posMsg.header.seq=0;

    motorController.setLaserMotorVelocity(laserMotorSpeed);

    //激光电机上下俯仰
    const short totalStage=7;//梯形速度控制的粒度，注意该值越大，换向所用的时间越长，并且可能导致卡位！
    short moveStage=totalStage;

    enum MOTOR_STATE{Up,Down,UpToDown,DownToUp};
    MOTOR_STATE motorState=Up;
    while(ros::ok()){
        motorController.getLaserMotorPosition(pos);
        posMsg.header.stamp=ros::Time::now();
        if(pos>laserMotorUpperBound || pos<laserMotorLowerBound){//限位保护
            ROS_ERROR("laser motor exceed! TotalStage too large or laserMotorThreshold too small!");
            break;
        }

        if(pos>laserMotorUpperBound-laserMotorThreshold && Up==motorState){
            motorState=UpToDown;
        }

        if(pos<laserMotorLowerBound+laserMotorThreshold && Down==motorState){
            motorState=DownToUp;
        }

        if(UpToDown==motorState){
            --moveStage;
            motorController.setLaserMotorVelocity(laserMotorSpeed*moveStage/totalStage);
            if(-totalStage==moveStage){
                motorState=Down;
            }
        }

        if(DownToUp==motorState){
            ++moveStage;
            motorController.setLaserMotorVelocity(laserMotorSpeed*moveStage/totalStage);
            if(totalStage==moveStage){
                motorState=Up;
            }
        }

        //TODO 使用PDO去定时发送消息
        posMsg.position.push_back(pos/scale);

        posMsg.position.push_back(laserMotorSpeed/scale*moveStage/totalStage);

        publisher.publish(posMsg);

        //std::cout<<"current pos " <<pos/scale <<" speed "<<laserMotorSpeed/scale*moveStage/totalStage<<std::endl;

        posMsg.position.clear();
        posMsg.header.seq++;

        //ros::Duration(0.05).sleep();

         usleep(time);

        ros::spinOnce();
    }
    motorController.setLaserMotorVelocity(0);

}

