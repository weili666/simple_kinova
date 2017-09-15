/** \file

Simple minded example of homing & moving a motor.
*/

// Comment this out to use EtherCAT
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <moveit/move_group_interface/move_group.h>
#define USE_CAN
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <stdio.h>
#include <thread>
#include "CML.h"
#include <unistd.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_agv_msgs/MoveBaseAGVAction.h>

#if defined( USE_CAN )
#include "can_kvaser.h"
#elif defined( WIN32 )
#include "ecat_winudp.h"
#else
#include "ecat_linux.h"
#endif
using namespace std;
// If a namespace has been defined in CML_Settings.h, this
// macros starts using it.
CML_NAMESPACE_USE();

/* local functions */
static void showerr( const Error *err, const char *str );

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 canNodeID = 1;                // CANopen node ID
int16 canNodeID2 = 2;
int16 canNodeID3 = 3;
int16 canNodeID4 = 4;
Amp amp,amp2,amp3,amp4;
ProfileConfigVel vel,vel2,vel3,vel4;
ProfileConfigTrap tra,tra2,tra3,tra4;
bool isReady;


/**************************************************
* Just home the motor and do a bunch of random
* moves.
**************************************************/

class MoveBaseAgvAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<move_base_agv_msgs::MoveBaseAGVAction> as_;
    std::string action_name_;
    move_base_agv_msgs::MoveBaseAGVFeedback feedback_;
    move_base_agv_msgs::MoveBaseAGVResult result_;
public:
    MoveBaseAgvAction(std::string name):as_(nh_, name, boost::bind(&MoveBaseAgvAction::executeCB, this, _1), false),action_name_(name)
    {
        as_.start();
    }

    ~MoveBaseAgvAction(void)
    {

    }

    void executeCB(const move_base_agv_msgs::MoveBaseAGVGoalConstPtr &goal)
    {
        ros::Rate r(1);
        bool success  = true;
        double D = 0.538;
        double R = 0.1;
        double scale  = 10000*32/360;//how many counts in 1 grade in number 1 motor;
        double scale2 = 10000*32/360;//how many counts in 1 grade in number 2 motor;
        double scale3 = 10000*123.863636/360;//how many counts in 1 grade in number 3 motor;
        double scale4 = 10000*88.3076923/360;//how many counts in 1 grade in number 4 motor;
        double w_L[goal->agv_poses.size()],w_R[goal->agv_poses.size()],w_T[goal->agv_poses.size()];

        double velocity,velocity2,velocity3,velocity4;
        double a,a2,a3,a4;
        const Error *err;

         for(int i = 0; i < goal->agv_poses.size()-1; i ++)
        {
          double delta_t = goal->agv_poses[i+1].time-goal->agv_poses[i].time;
          w_L[i] = (180/M_PI)*((sqrt((goal->agv_poses[i+1].pose_g.position[0]-goal->agv_poses[i].pose_g.position[0])*(goal->agv_poses[i+1].pose_g.position[0]-goal->agv_poses[i].pose_g.position[0])+(goal->agv_poses[i+1].pose_g.position[1]-goal->agv_poses[i].pose_g.position[1])*(goal->agv_poses[i+1].pose_g.position[1]-goal->agv_poses[i].pose_g.position[1]))-(goal->agv_poses[i+1].pose_g.position[2]-goal->agv_poses[i].pose_g.position[2])*D/2)/(R*delta_t));
          w_R[i] = (180/M_PI)*((sqrt((goal->agv_poses[i+1].pose_g.position[0]-goal->agv_poses[i].pose_g.position[0])*(goal->agv_poses[i+1].pose_g.position[0]-goal->agv_poses[i].pose_g.position[0])+(goal->agv_poses[i+1].pose_g.position[1]-goal->agv_poses[i].pose_g.position[1])*(goal->agv_poses[i+1].pose_g.position[1]-goal->agv_poses[i].pose_g.position[1]))+(goal->agv_poses[i+1].pose_g.position[2]-goal->agv_poses[i].pose_g.position[2])*D/2)/(R*delta_t));
          w_T[i] = (180/M_PI)*(goal->agv_poses[i+1].pose_g.position[3]-goal->agv_poses[i].pose_g.position[3])/delta_t;
        }

        w_L[goal->agv_poses.size()-1] = 0;
        w_R[goal->agv_poses.size()-1] = 0;
        w_T[goal->agv_poses.size()-1] = 0;

        amp.sdo.SetTimeout(10);//1000 represent 2 seconds
        amp2.sdo.SetTimeout(10);//1000 represent 2 seconds
        amp3.sdo.SetTimeout(10);//1000 represent 2 seconds
        amp4.sdo.SetTimeout(10);//1000 represent 2 seconds

        feedback_.agv_velocities.resize(goal->agv_poses.size()-1);
        result_.agv_velocities.resize(goal->agv_poses.size()-1);

        for(int i = 0; i < goal->agv_poses.size()-1; i ++)
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
              ROS_INFO("%s: Preempted", action_name_.c_str());
              // set the action state to preempted
              as_.setPreempted();
              success = false;
              break;
            }
            double delta_t = goal->agv_poses[i+1].time-goal->agv_poses[i].time;
            amp.SetTargetVel(w_L[i]*scale);
            amp2.SetTargetVel(w_R[i]*scale2);
            amp3.SetTargetVel(-w_T[i]*scale3);
            amp4.SetTargetVel(w_T[i]*scale4);

            amp.SetAmpMode( AMPMODE_CAN_VELOCITY );
            amp2.SetAmpMode( AMPMODE_CAN_VELOCITY );
            amp3.SetAmpMode( AMPMODE_CAN_VELOCITY );
            amp4.SetAmpMode( AMPMODE_CAN_VELOCITY );

            amp.sdo.SetTimeout(int(250*delta_t));//1000 represent 2 seconds
            amp2.sdo.SetTimeout(int(250*delta_t));//1000 represent 2 seconds
            amp3.sdo.SetTimeout(int(250*delta_t));//1000 represent 2 seconds
            amp4.sdo.SetTimeout(int(250*delta_t));//1000 represent 2 seconds
            //sleep(delta_t);

            err = amp.StartMove(true);
            err = amp2.StartMove(true);
            err = amp3.StartMove(true);
            err = amp4.StartMove(true);

            amp.GetVelocityActual(velocity);
            amp2.GetVelocityActual(velocity2);
            amp3.GetVelocityActual(velocity3);
            amp4.GetVelocityActual(velocity4);

            feedback_.agv_velocities[i].w_L = velocity;
            feedback_.agv_velocities[i].w_R = velocity2;
            feedback_.agv_velocities[i].w_T = velocity3;

            amp.GetHomeAccel(a);
            amp2.GetHomeAccel(a2);
            amp3.GetHomeAccel(a3);
            amp4.GetHomeAccel(a4);

            cout<<"vel:"<<velocity<<", "<<w_L[i-1]<<" accel:"<<a<<endl;
            cout<<"vel2:"<<velocity2<<", "<<w_R[i-1]<<" accel2:"<<a2<<endl;
            cout<<"vel3:"<<velocity3<<", "<<w_T[i-1]<<" accel3:"<<a3<<endl;
            cout<<"vel4:"<<velocity4<<", "<<w_T[i-1]<<" accel4:"<<a4<<endl;
            cout<<"time:"<<delta_t<<endl;
            r.sleep();

        }

        amp.SetTargetVel(0);
        amp2.SetTargetVel(0);
        amp3.SetTargetVel(0);
        amp4.SetTargetVel(0);

        amp.SetAmpMode( AMPMODE_CAN_VELOCITY );
        amp2.SetAmpMode( AMPMODE_CAN_VELOCITY );
        amp3.SetAmpMode( AMPMODE_CAN_VELOCITY );
        amp4.SetAmpMode( AMPMODE_CAN_VELOCITY );

        amp.sdo.SetTimeout(500);//1000 represent 2 seconds
        amp2.sdo.SetTimeout(500);//1000 represent 2 seconds
        amp3.sdo.SetTimeout(500);//1000 represent 2 seconds
        amp4.sdo.SetTimeout(500);//1000 represent 2 seconds

        //sleep(1);
        err = amp.StartMove(true);
        err = amp2.StartMove(true);
        err = amp3.StartMove(true);
        err = amp4.StartMove(true);

        as_.publishFeedback(feedback_);
        if(success)
        {
          result_.agv_velocities = feedback_.agv_velocities;
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          as_.setSucceeded(result_);
        }
    }
};

int main( int argc,char **argv )
{
    ros::init(argc, argv, "move_agv");
    ros::NodeHandle nh;
    cml.SetDebugLevel( LOG_EVERYTHING );
//====Create an object used to access the low level CAN network.====//

#if defined( USE_CAN )
   KvaserCAN hw( "CAN0" );
   KvaserCAN hw2( "CAN0" );
   KvaserCAN hw3( "CAN0" );
   KvaserCAN hw4( "CAN0" );
   hw.SetBaud( canBPS );
   hw2.SetBaud( canBPS );
   hw3.SetBaud( canBPS );
   hw4.SetBaud( canBPS );
   printf("hello world 1\n");
#elif defined( WIN32 )
   WinUdpEcatHardware hw( "eth0" );
#else
   LinuxEcatHardware hw( "eth0" );
#endif
//==================================================================//

//============== Open the network object==============//
#if defined( USE_CAN )
   CanOpen net;
   CanOpen net2;
   CanOpen net3;
   CanOpen net4;
   printf("hello world 2\n");
#else
   EtherCAT net;
   EtherCAT net2;
#endif
   printf("hello world 3\n");
   const Error *err = net.Open( hw );
   err = net2.Open( hw2 );
   err = net3.Open( hw3 );
   err = net4.Open( hw4 );
   showerr( err, "Opening network" );
//====================================================//

//===Initialize the amplifier using default settings===//

   printf( "Doing init\n" );
   err = amp.Init( net, canNodeID );
   err = amp2.Init( net2, canNodeID2 );
   err = amp3.Init( net3, canNodeID3 );
   err = amp4.Init( net4, canNodeID4 );
   showerr( err, "Initting amp" );
   //getchar();
//=====================================================//



//=====================execute the trajectory=========================//

MoveBaseAgvAction move_agv("move_base_agv");
ros::spin();

//====================================================================//

   return 0;

}

/**************************************************/

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      exit(1);
   }
}
