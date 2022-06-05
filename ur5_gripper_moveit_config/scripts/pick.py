#! /usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    roscpp_initialize,
    roscpp_shutdown,
)
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped


roscpp_initialize(sys.argv)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("ur5_arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")


p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.7
p.pose.position.y = -0.4
p.pose.position.z = 0.85
scene.add_box("box", p, (0.075, 0.075, 0.075))

p.pose.position.x = 0.8
p.pose.position.y = 0
p.pose.position.z = 0.181
scene.add_box("table", p, (0.8, 0.8, 0.01))
"""
box_pose = geometry_msgs.msg.Pose()
box_pose.orientation.w = 0
box_pose.position.x = 0.6
box_pose.position.y = -0.2
box_pose.position.z = 0,773999  
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

table_pose = geometry_msgs.msg.Pose()

table_pose.position.x = 0.8
table_pose.position.y = 0.0
table_pose.position.z = 0.181
table_name = "table"
scene(table_name, table_pose, size=(0.8, 0.8, 0.01))
"""
# Put the arm in the start position
arm_group.set_named_target("home")
plan1 = arm_group.go()

# Open the gripper
hand_group.set_named_target("open")
plan2 = hand_group.go()



# put the arm at the 1st grasping position
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0
pose_target.orientation.x = 0
pose_target.orientation.y = 0
pose_target.orientation.z = 0
pose_target.position.x = -0.2
pose_target.position.y = 0.4
pose_target.position.z = 0.05
arm_group.set_pose_target(pose_target)
plan1 = arm_group.plan()
plan1 = arm_group.go()

pose_target.position.x += 0.1
arm_group.set_pose_target(pose_target)
plan2 = arm_group.plan()
plan2 = arm_group.go()

# close the gripper

hand_group.set_named_target("closed")
plan3 = hand_group.go()

# put the arm at the 3rd grasping position
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan4 = arm_group.plan()
plan4 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()

