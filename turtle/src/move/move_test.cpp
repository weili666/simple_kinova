

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
#include <move_base_agv_msgs/MoveBaseAGVAction.h>
#include <move_base_agv_msgs/AgvPose.h>

using namespace std;

class parameter_base
{
public:
    parameter_base( actionlib::SimpleActionClient<move_base_agv_msgs::MoveBaseAGVAction> *ac_, const move_base_agv_msgs::MoveBaseAGVGoal &goal_)
    {
        ac = ac_;
        goal = goal_;
        ac->waitForServer();
    }

    actionlib::SimpleActionClient<move_base_agv_msgs::MoveBaseAGVAction> *ac;

    move_base_agv_msgs::MoveBaseAGVGoal goal;
};


int main(int argc,char **argv)
{
    ros::init(argc, argv, "kinova_agv");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    typedef actionlib::SimpleActionClient<move_base_agv_msgs::MoveBaseAGVAction> MoveBaseAGVActionClient;
    MoveBaseAGVActionClient*      ac = new MoveBaseAGVActionClient("move_base_agv",true);

    move_base_agv_msgs::MoveBaseAGVGoal goal;

    for(unsigned int i=0;i<3;i++)
    {
      sensor_msgs::JointState pose_g;

      pose_g.position.push_back(i);
      pose_g.position.push_back(0);
      pose_g.position.push_back(0);
      pose_g.position.push_back(0);

      move_base_agv_msgs::AgvPose agv_pose;
      agv_pose.pose_g = pose_g;
      agv_pose.time = i;

      goal.agv_poses.push_back(agv_pose);
    }

    parameter_base pb(ac,goal);
    pb.ac->sendGoal(pb.goal);
}
