#!/usr/bin/env python  
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('lab3', anonymous=True)

# The "RobotCommander" object is an interface to Baxter (or any robot) as a whole.
robot = moveit_commander.RobotCommander()

# This is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# This is an interface to one group of joints.  In our case, we want to use the "right_arm".
#We will use this to plan and execute motions
group = moveit_commander.MoveGroupCommander("left_arm")

# This will give you the frame that the robot is attached, for Baxter, this is the "world" frame
group.get_planning_frame()

# It is also possible to find out what the end-effector of the robot
group.get_end_effector_link()


pose_target = geometry_msgs.msg.Pose()
# 3D point and quaternion (same as previous lab, e.g. 0.644, 0.0, 0.0 for XYZ and
# -0.381, 0.923, -0.015, 0.052 for the XYZW components of the quaternion).
# NOTE: This point might fail if it is close or in the box! Choose a different point if that's the case

# pose_target is initialised with zeros in all entries so let's popualate the quaternion
pose_target.orientation.x = -0.381
pose_target.orientation.y = 0.923
pose_target.orientation.z = -0.015
pose_target.orientation.w = 0.052


pose_target.position.x = 0.644
pose_target.position.y = 0.0
pose_target.position.z = 0.0

# Now, add your Pose msg to the group's pose target
group.set_pose_target(pose_target)

# and compute the plan!
plan = group.plan()
print(plan)

ret = group.execute(plan)

waypoints = []

# start with the current pose
waypoints.append(group.get_current_pose().pose)

# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = 1.0
wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

# second move down
wpose.position.z -= 0.10
waypoints.append(copy.deepcopy(wpose))

# third move to the side
wpose.position.y += 0.05
waypoints.append(copy.deepcopy(wpose))

# The cartesian path will be interpolated at a resolution of 1 cm (i.e. 0.01 as the second argument below
# for the end-effector. The third argument is to disable the "jump threshold" see:
# http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a4a3cfd21dd94bcc6991797e474c4d7f3
(plan_cartesian, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

# If everything worked out fine, execute the above plan!
group.execute(plan_cartesian)

