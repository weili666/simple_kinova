#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
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
//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
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

int main(int argc, char **argv)
{
    ros::init(argc,argv,"move_kinova_example");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber jointStateSubscriber;



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
    KDL::JntArray  q_;
    KDL::Frame     x_;
    KDL::Jacobian  J_;
    q_.resize(6);

    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(jaco_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(jaco_chain));

    moveit::planning_interface::MoveGroup group("arm_chain_group");


    //=====================after plan and public======================//

    double t;
    double T=6.28;
    int N=63;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::RobotTrajectory trajectory_msg_;
    trajectory_msgs::JointTrajectory trajectory =  trajectory_msg_.joint_trajectory;
    trajectory.points.resize(N);
    trajectory.joint_names.push_back("jaco_joint_1");
    trajectory.joint_names.push_back("jaco_joint_2");
    trajectory.joint_names.push_back("jaco_joint_3");
    trajectory.joint_names.push_back("jaco_joint_4");
    trajectory.joint_names.push_back("jaco_joint_5");
    trajectory.joint_names.push_back("jaco_joint_6");

    int num_of_joints=6;
    double A=0.6;
    double B=0.6;
    double C=3.6;

    for(int i=0;i<N;i++)
    {
        t=i*T/N;
        trajectory.points[i].positions.resize(num_of_joints);
        trajectory.points[i].velocities.resize(num_of_joints);
        trajectory.points[i].accelerations.resize(num_of_joints);
        trajectory.points[i].positions[0]= A*cos(t)-A*cos(2*t)-1.4735840227881702;
        trajectory.points[i].velocities[0]= -A*sin(t)+A*2*sin(2*t);
        trajectory.points[i].accelerations[0]= -A*cos(t)+A*4*cos(2*t);
        trajectory.points[i].positions[1]= B*cos(2*t)-B*cos(4*t)+2.919063173960516;
        trajectory.points[i].velocities[1]= -B*2*sin(2*t)+B*4*sin(4*t);
        trajectory.points[i].accelerations[1]= -B*4*cos(t)+B*16*cos(4*t);
        trajectory.points[i].positions[2]= C*cos(0.25*t)-C*cos(0.5*t)+1.0135101613037494;
        trajectory.points[i].velocities[2]= -C*0.25*sin(0.25*t)+C*0.5*sin(0.5*t);
        trajectory.points[i].accelerations[2]= -C*0.25*0.25*cos(t)+C*0.25*cos(0.5*t);
        trajectory.points[i].positions[3]= -2.0836849337533323;
        trajectory.points[i].velocities[3]= 0;
        trajectory.points[i].accelerations[3]= 0;
        trajectory.points[i].positions[4]= 1.443466866652682;
        trajectory.points[i].velocities[4]= 0;
        trajectory.points[i].accelerations[4]= 0;
        trajectory.points[i].positions[5]= 1.3149469735032022;
        trajectory.points[i].velocities[5]= 0;
        trajectory.points[i].accelerations[5]= 0;

    }
    moveit::planning_interface::MoveGroup::Plan mani_plan;
    trajectory_msg_.joint_trajectory=trajectory;
    mani_plan.trajectory_=trajectory_msg_;
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start=mani_plan.start_state_;
    display_trajectory.trajectory.push_back(mani_plan.trajectory_);
    display_publisher.publish(display_trajectory);

    bool _return=group.execute(mani_plan);
    ROS_INFO("Visualizing simultanious plan manifold %s",_return?" ":"Failed");
    sleep(5.0);

    ros::spin();
    return 0;
}
