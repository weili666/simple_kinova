#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//Used to read pre-stored trajectory file
//C++ file output/input
#include <iostream>
#include <fstream>
//Ros package
#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <wpi_jaco_msgs/GetAngularEffort.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ros::ServiceClient client_effort = node_handle.serviceClient<wpi_jaco_msgs::GetAngularEffort>("jaco_arm/get_angular_effort");
  ros::ServiceClient client_posi = node_handle.serviceClient<wpi_jaco_msgs::GetAngularPosition>("jaco_arm/get_angular_position");
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm");

//Prapare the arm for the identification work--------------------------------------------------------//
  // std::vector<double> group_variable_values;
  // group_variable_values.clear();
  // group_variable_values.push_back(2.433);
  // group_variable_values.push_back(3.14);
  // group_variable_values.push_back(3.14);
  // group_variable_values.push_back(3.14);
  // group_variable_values.push_back(3.14);
  // group_variable_values.push_back(3.14);
  // group.setJointValueTarget(group_variable_values);

  // moveit::planning_interface::MoveGroup::Plan my_plan2;
  // bool success_plan2 = group.plan(my_plan2);
  // bool success_execute2 = group.execute(my_plan2);





  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 0;
  // target_pose1.orientation.x = 0;
  // target_pose1.orientation.y = 0.707;
  // target_pose1.orientation.z = 0.707;
  // target_pose1.position.x = 0.1;
  // target_pose1.position.y = -0.2;
  // target_pose1.position.z = 0.4;
  // group.setPoseTarget(target_pose1);

  // moveit::planning_interface::MoveGroup::Plan my_plan1;
  // bool success_plan1 = group.plan(my_plan1);
  // bool success_execute1 = group.execute(my_plan1);

//Read data from the pre-stored txt file---------------------------------------------------------------//
  std::string string_line;
  std::ifstream infile;
  std::string package_path_ = ros::package::getPath( "jaco_dynamic_ident" ) + "/joint_angle_file/joint_angle_1203.txt";
  infile.open(package_path_.c_str());
  std::vector< std::vector<double> > joint_vector;
  while (std::getline(infile, string_line)) // To get you all the lines.
  {
    std::vector<double> joint;
    joint.clear();
    double q1, q2, q3, q4, q5, q6;
    sscanf(string_line.c_str(), "%lf %lf %lf %lf %lf %lf ", &q1, &q2, &q3, &q4, &q5, &q6);
    if (q2 < 3.14)
    {
      joint.push_back(q1 + 1.57);
    }
    else
    {
      joint.push_back(q1 + 3.14);
    }
    joint.push_back(q2);
    joint.push_back(q3);
    joint.push_back(q4);
    joint.push_back(q5);
    joint.push_back(q6);
    joint_vector.push_back(joint);

  }
  infile.close();
//Plan for these point to know if these point is reachable-----------------------------------------//
  int num_threads = boost::thread::hardware_concurrency();
  group.setNumPlanningAttempts(num_threads);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(5.0);
  int success_number = 0;
  std::vector< std::vector<double> > joint_success_vector;
  std::vector< std::vector<double> > joint_success_effort;
  std::vector<int> index_number;



  wpi_jaco_msgs::GetAngularPosition srv_posi;
  wpi_jaco_msgs::GetAngularEffort srv_effort;
  //for (unsigned int i = 0; i < 1; i++)//
  for (unsigned int i = 896; i < joint_vector.size(); i++)
  {
    group.setJointValueTarget(joint_vector[i]);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool plan_success = group.plan(my_plan);

    if (plan_success)
    {
      ROS_INFO_STREAM("Motion plan Succeeded for position: " << i);
      bool execute_success = group.execute(my_plan);
      if (execute_success)
      {
        ROS_INFO_STREAM("Execution Succeeded for pisition: " << i);
        success_number++;
        //Store the joint effort and position data
        //sleep 2 second before get position and effort
        index_number.push_back(i);
        ros::Duration(2.5).sleep();
        if (client_posi.call(srv_posi) && client_effort.call(srv_effort))
        {
          // std::vector<double> joint_p;
          // joint_p.clear();
          // joint_p.push_back(srv_posi.response.pos[0]);
          // joint_p.push_back(srv_posi.response.pos[1]);
          // joint_p.push_back(srv_posi.response.pos[2]);
          // joint_p.push_back(srv_posi.response.pos[3]);
          // joint_p.push_back(srv_posi.response.pos[4]);
          // joint_p.push_back(srv_posi.response.pos[5]);
          joint_success_vector.push_back(joint_vector[i]);

          std::vector<double> joint_e;
          joint_e.clear();
          joint_e.push_back(srv_effort.response.eff[0]);
          joint_e.push_back(srv_effort.response.eff[1]);
          joint_e.push_back(srv_effort.response.eff[2]);
          joint_e.push_back(srv_effort.response.eff[3]);
          joint_e.push_back(srv_effort.response.eff[4]);
          joint_e.push_back(srv_effort.response.eff[5]);
          joint_success_effort.push_back(joint_e);
        }

        // std::vector<double> joint_e;
        // joint_e.clear();
        // joint_e.resize(6);
        // joint_e.assign(joint_e.size(), 0);
        // unsigned s_time = 0;
        // for (unsigned int i = 0; i < 20; i++)
        // {
        //   if (client_effort.call(srv_effort))
        //   {
        //     for (unsigned int j = 0; j < joint_e.size(); j++)
        //     {
        //       joint_e[j] += (srv_effort.response.eff[j]);
        //     }
        //     s_time++;
        //   }
        //   //ros::Duration(0.3).sleep();
        // }
        // for (unsigned int j = 0; j < joint_e.size(); j++)
        // {
        //   joint_e[j] /= s_time;
        // }
        // joint_success_effort.push_back(joint_e);

      }
      else
      {
        ROS_ERROR_STREAM("execution Failed for position: " << i);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Motion plan Failed for position: " << i);
    }
  }

  ROS_INFO_STREAM("Success rate: " << double(success_number) / joint_vector.size());

  std::ofstream outfile2("src/result.txt", std::ofstream::app);
  if (!outfile2)
  {
    std::cout << "Unable to open outfile";
    exit(1); // terminate with error
  }

  for (unsigned int i = 0; i < joint_success_vector.size(); i++)
  {
    outfile2 << index_number[i] << "|" << joint_success_vector[i][0] << "|" << joint_success_vector[i][1]
             << "|" << joint_success_vector[i][2]  << "|" << joint_success_vector[i][3]
             << "|" << joint_success_vector[i][4]  << "|" << joint_success_vector[i][5]
             << "|" << joint_success_effort[i][0] << "|" << joint_success_effort[i][1]
             << "|" << joint_success_effort[i][2]  << "|" << joint_success_effort[i][3]
             << "|" << joint_success_effort[i][4]  << "|" << joint_success_effort[i][5]
             << std::endl;
  }
  outfile2.close();

  ros::shutdown();
  return 0;
}
