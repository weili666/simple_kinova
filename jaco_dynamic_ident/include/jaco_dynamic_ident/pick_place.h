#ifndef Dynamic_Iden_H_
#define Dynamic_Iden_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit_msgs/GetPositionIK.h>
#include <industrial_utils/param_utils.h>
#include <actionlib/client/action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/DisplayTrajectory.h>

// =============================== aliases ===============================
typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

namespace jaco_dynamic
{

class Dynamic_Iden
{
public:
    // =============================== constructor =====================================
    Dynamic_Iden()
    {

    }/**<default constructor */

    // =============================== global members =====================================
    ros::ServiceClient motion_plan_client;/**<client for motion plan:generate trajectory input start,goal and other user-defined parameter */
    MoveGroupPtr move_group_ptr;/**<pointer for home robot move group */
    TransformListenerPtr transform_listener_ptr;/**<pointer for home robot transform system*/

    // =============================== Task Functions ===============================
    bool create_motion_plan(const geometry_msgs::Pose &pose_target,
                            const moveit_msgs::RobotState &start_robot_state, move_group_interface::MoveGroup::Plan &plan);

};

}


#endif /* Dynamic_Iden_H_ */

