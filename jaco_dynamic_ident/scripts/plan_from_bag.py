import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rosbag
from std_msgs.msg import Int32, String, Float64

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
group.set_planner_id("RRTConnectkConfigDefault")
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

print "============ Starting tutorial "
bag = rosbag.Bag('test_plan.bag')
trajectory_push = moveit_msgs.msg.RobotTrajectory()
trajectory_retreat = moveit_msgs.msg.RobotTrajectory()
for topic, msg, t in bag.read_messages(topics=['trajectory_push']):
     trajectory_push = msg
for topic, msg, t in bag.read_messages(topics=['trajectory_retreat']):
     trajectory_retreat = msg
bag.close()

print "============ Printing  for joint target "
group_variable_values_end = group.get_current_joint_values()
print "============ Joint values: ", group_variable_values_end
group_variable_values_end[0] = trajectory_push.joint_trajectory.points[0].positions[0];
group_variable_values_end[1] = trajectory_push.joint_trajectory.points[0].positions[1];
group_variable_values_end[2] = trajectory_push.joint_trajectory.points[0].positions[2];
group_variable_values_end[3] = trajectory_push.joint_trajectory.points[0].positions[3];
group_variable_values_end[4] = trajectory_push.joint_trajectory.points[0].positions[4];
group_variable_values_end[5] = trajectory_push.joint_trajectory.points[0].positions[5];
print "============ Joint values: ", group_variable_values_end
group.set_joint_value_target(group_variable_values_end)

plan1 = group.plan()
#group.go(wait=True)
group.execute(plan1)

rospy.sleep(3)
plan2 = trajectory_push
group.execute(plan2)
rospy.sleep(10)

print "============ Joint values: ", group_variable_values_end
#group.set_joint_value_target(group_variable_values_end)
#plan3 = group.plan()
plan3 = trajectory_retreat
#group.go(wait=True)
group.execute(plan3)
