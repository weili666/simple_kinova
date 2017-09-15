#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
//joint state msg related
#include <wpi_jaco_msgs/JointPosiCartForce.h>
#include <std_msgs/Float32MultiArray.h>
//KDL related
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

//boost related
#include <boost/scoped_ptr.hpp>
#include <boost/make_shared.hpp>

//torque command message related
#include <wpi_jaco_msgs/AngularTorqueCommand.h>
#include <wpi_jaco_msgs/CartesianForceCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/EulerToQuaternion.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/StartForceControl.h>
#include <wpi_jaco_msgs/StopForceControl.h>

//tf related
#include <tf/transform_listener.h>

//finger action related
#include <actionlib/client/simple_action_client.h>
#include <wpi_jaco_msgs/SetFingersPositionAction.h>


//boost
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

using namespace std;

void JointDataCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    ofstream fout,fout2;
    fout.open("$(find simple_kinova)/src/system_identification/data/joint.txt",ios::app|ios::out);
    fout2.open("$(find simple_kinova)/src/system_identification/data/effort.txt",ios::app|ios::out);
    vector<double> joint_vector,effort_vector;
    joint_vector.assign(joint_state->position.begin(),joint_state->position.end());
    effort_vector.assign(joint_state->effort.begin(),joint_state->effort.end());
    fout<<joint_vector[0]<<setw(12)<<joint_vector[1]<<setw(12)<<joint_vector[2]<<setw(12)<<joint_vector[3]<<setw(12)<<joint_vector[4]<<setw(12)<<joint_vector[5]<<"\n";
    cout<<"joint :"<<joint_vector[0]<<" , "<<joint_vector[1]<<" , "<<joint_vector[2]<<" , "<<joint_vector[3]<<" , "<<joint_vector[4]<<" , "<<joint_vector[5]<<endl;
    fout2<<effort_vector[0]<<setw(12)<<effort_vector[1]<<setw(12)<<effort_vector[2]<<setw(12)<<effort_vector[3]<<setw(12)<<effort_vector[4]<<setw(12)<<effort_vector[5]<<"\n";
    cout<<"effort :"<<effort_vector[0]<<" , "<<effort_vector[1]<<" , "<<effort_vector[2]<<" , "<<effort_vector[3]<<" , "<<effort_vector[4]<<" , "<<effort_vector[5]<<endl;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"move_kinova_example2");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber jointStateSubscriber;
    ros::Subscriber jointAccelSubscriber;


    std::cout << "Getting the urdf..." << std::endl;
    std::string robot_description;
    std::string robot_param;

    nh.searchParam("robot_description", robot_param);
    nh.param<std::string>(robot_param, robot_description, "");
    //check the param server return
    if (robot_description.empty())
    {
        std::cout << "Failed to retreive robot urdf" << std::endl;
        return 0;
    }
    //parse the KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description, kdl_tree))
    {
        std::cout << "Failed to parse the kdl tree" << std::endl;
        return 0;
    }

    //parse the KDL chain
    KDL::Chain jaco_chain;
    std::string arm_name_;
    arm_name_="jaco";
    nh.param("wpi_jaco/arm_name", arm_name_, std::string("jaco"));
    if (!kdl_tree.getChain(arm_name_+"_link_base", arm_name_+"_link_hand", jaco_chain))
    {
        std::cout << "Failed to parse the kdl chain" << std::endl;
        return 0;
    }
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr = boost::make_shared<KDL::Chain>(jaco_chain);
    std::cout << "KDL chain has " << kdl_chain_ptr->getNrOfSegments() << " segments and " << kdl_chain_ptr->getNrOfJoints() << " joints." << std::endl;

    std::cout << "Joints: ";
    for (unsigned int i = 0; i < kdl_chain_ptr->getNrOfSegments(); i++)
        std::cout << kdl_chain_ptr->segments.at(i).getJoint().getName() << " ";
    std::cout << std::endl;
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(jaco_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(jaco_chain));

    cout<<"data record:"<<endl;
    jointStateSubscriber = nh.subscribe("/jaco_arm/joint_states",1000,JointDataCallback);


    ros::spin();

    return 0;
}
