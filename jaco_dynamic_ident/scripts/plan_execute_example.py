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
                                    moveit_msgs.msg.DisplayTrajectory,queue_size=10)

print "============ Starting tutorial "

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Reference frame: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
print "============"
group.clear_pose_targets()
print "============ Printing  for joint target 1"
group_variable_values = group.get_current_joint_values()
print "============ Joint values: ", group_variable_values
group_variable_values[0] = -1.60;
group_variable_values[1] = 4.54;
group_variable_values[2] = 1.82;
group_variable_values[3] = 2.86;
group_variable_values[4] = 1.298;
group_variable_values[5] = 2.51;
group.set_joint_value_target(group_variable_values)

plan1 = group.plan()
group.execute(plan1)

print "============ Printing  for joint target 2"
group_variable_values_end = group.get_current_joint_values()
print "============ Joint values: ", group_variable_values_end
group_variable_values_end[0] = -1.60;
group_variable_values_end[1] = 4.56;
group_variable_values_end[2] = 1.87;
group_variable_values_end[3] = 2.88;
group_variable_values_end[4] = 1.25;
group_variable_values_end[5] = 2.53;
group.set_joint_value_target(group_variable_values_end)

plan2 = group.plan()
#group.go(wait=True)
group.execute(plan2)

print "============ Printing  for cartesian target 1"
group_variable_values_end = group.get_current_joint_values()
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = -0.5
pose_target.orientation.w = 0.5
pose_target.position.x = 1.70
pose_target.position.y = 0.35
pose_target.position.z = 0.87
group.set_pose_target(pose_target)

plan3 = group.plan()
#group.go(wait=True)
group.execute(plan3)

print "============ Printing  for cartesian target 2"
# group_variable_values_end = group.get_current_joint_values()
# pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = -0.5
pose_target.orientation.w = 0.5
pose_target.position.x = 1.85
pose_target.position.y = 0.35
pose_target.position.z = 0.87
group.set_pose_target(pose_target)

plan4 = group.plan()
#group.go(wait=True)
group.execute(plan4)

print "============ Printing  for cartesian target 3"
# group_variable_values_end = group.get_current_joint_values()
# pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.5
pose_target.orientation.y = -0.5
pose_target.orientation.z = -0.5
pose_target.orientation.w = 0.5
pose_target.position.x = 1.70
pose_target.position.y = 0.35
pose_target.position.z = 0.87
group.set_pose_target(pose_target)

plan5 = group.plan()
#group.go(wait=True)
group.execute(plan5)



bag = rosbag.Bag('test_plan.bag', 'w')

try:
    trajectory_push = moveit_msgs.msg.RobotTrajectory()
    trajectory_push = plan4
    # print "============ Printing  trajectory before time modification"
    # print trajectory_push
    # for point in trajectory_push.joint_trajectory.points:
    #             point.time_from_start = rospy.Duration.from_sec(point.time_from_start.to_sec()*2.5)
    # print "============ Printing  trajectory after time modification"
    # print trajectory_push

    trajectory_retreat = moveit_msgs.msg.RobotTrajectory()
    trajectory_retreat = plan5
    # print "============ Printing  trajectory before time modification"
    # print trajectory_retreat
    # for point in trajectory_retreat.joint_trajectory.points:
    #             point.time_from_start = rospy.Duration.from_sec(point.time_from_start.to_sec()*2.5)
    # print "============ Printing  trajectory after time modification"
    # print trajectory_retreat

    bag.write('trajectory_push', trajectory_push)
    bag.write('trajectory_retreat', trajectory_retreat)
finally:
    bag.close()







