#!/usr/bin/env python

import sys
import copy
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

# *************************** Solution
# Generates a wave from the current pose
def wave_generator(group, initial_pose_left, initial_pose_right, step=15, radius_wave=0.3):
    # start with the current pose
    c_pose_left = copy.deepcopy(initial_pose_left)
    c_pose_right = copy.deepcopy(initial_pose_right)
    wave = []
    wave_reverse = []
    
    for t in range(45,135,step):
        y_wave = initial_pose_left.position.y + radius_wave * np.cos(np.deg2rad(t))
        z_wave = initial_pose_left.position.z + radius_wave * np.sin(np.deg2rad(t))
        c_pose_left.position.y = y_wave
        c_pose_left.position.z = z_wave
        
        y_wave = initial_pose_right.position.y + radius_wave * np.cos(np.deg2rad(t))
        z_wave = initial_pose_right.position.z + radius_wave * np.sin(np.deg2rad(t))
        c_pose_right.position.y = y_wave
        c_pose_right.position.z = z_wave
        wave.append((copy.deepcopy(c_pose_left), copy.deepcopy(c_pose_right)))
    
    # Now the other way
    for t in range(135,45,-1*step):
        y_wave = initial_pose_left.position.y + radius_wave * np.cos(np.deg2rad(t))
        z_wave = initial_pose_left.position.z + radius_wave * np.sin(np.deg2rad(t))
        c_pose_left.position.y = y_wave
        c_pose_left.position.z = z_wave
        
        y_wave = initial_pose_right.position.y + radius_wave * np.cos(np.deg2rad(t))
        z_wave = initial_pose_right.position.z + radius_wave * np.sin(np.deg2rad(t))
        c_pose_right.position.y = y_wave
        c_pose_right.position.z = z_wave
        wave_reverse.append((copy.deepcopy(c_pose_left), copy.deepcopy(c_pose_right)))

    return wave, wave_reverse
# ***************************

def moveit_baxter():
    # *************************** From Lab 3
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('lab3', anonymous=True)

    # The "RobotCommander" object is an interface to Baxter (or any robot) as a whole.
    robot = moveit_commander.RobotCommander()

    # This is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    
    # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
    #We will use this to plan and execute motions
    group = moveit_commander.MoveGroupCommander("both_arms")
    #right_group = moveit_commander.MoveGroupCommander("right_arm")
    
    print group.get_planning_frame()
    print group.get_end_effector_link()
    
    initial_pose_left = geometry_msgs.msg.Pose()
    # 3D point and quaternion (same as previous lab, e.g. 0.644, 0.3, 0.8485 for XYZ and
    # -0.381, 0.923, -0.015, 0.052 for the XYZW components of the quaternion).
    # NOTE: This point might fail if it is close or in the box! Choose a different point if that's the case

    # initial_pose_left is initialised with zeros in all entries so let's popualate the quaternion
    initial_pose_left.orientation.x = -0.381
    initial_pose_left.orientation.y = 0.923
    initial_pose_left.orientation.z = -0.015
    initial_pose_left.orientation.w = 0.052
    initial_pose_left.position.x = 0.644
    initial_pose_left.position.y = 0.3
    initial_pose_left.position.z = 0.8485

    # Now, add your Pose msg to the group's pose target
    group.set_pose_target(initial_pose_left, end_effector_link = "left_gripper")
    
    # Now for right group
    initial_pose_right = copy.deepcopy(initial_pose_left)
    initial_pose_right.position.y *= -1
    group.set_pose_target(initial_pose_right, end_effector_link = "right_gripper")
    
    # and compute the plan!
    plan = group.plan()
    ret = group.execute(plan)
    # ***************************

    # *************************** Solution
    # Generate wave motion and return a cartesian plan
    wave, wave_reverse = wave_generator(group, initial_pose_left, initial_pose_right, step=10, radius_wave=0.1)
    
    # Now plan and execute
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        for w in wave:
            (pose_left, pose_right) = w
            print "********"
            print pose_left.position
            group.set_pose_target(pose_left, end_effector_link = "left_gripper")
            group.set_pose_target(pose_right, end_effector_link = "right_gripper")
            plan = group.plan()
            group.execute(plan)

        for w in wave_reverse:
            (pose_left, pose_right) = w
            print "********"
            print pose_left.position
            group.set_pose_target(pose_left, end_effector_link = "left_gripper")
            group.set_pose_target(pose_right, end_effector_link = "right_gripper")
            plan = group.plan()
            group.execute(plan)
            
        r.sleep()
        
    # ***************************

if __name__ == '__main__':
    import sys
    sys.exit(moveit_baxter())
