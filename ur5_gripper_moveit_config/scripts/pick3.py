#!/usr/bin/env python

import sys
import rospy
import tf
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    roscpp_initialize,
    roscpp_shutdown,
    MoveGroupCommander
)
from geometry_msgs.msg import PoseStamped, geometry_msgs

if __name__ == "__main__":

    roscpp_initialize(sys.argv)
    rospy.init_node("pick3", anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm_group = MoveGroupCommander("ur5_arm")
    hand_group = MoveGroupCommander("gripper")
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("box")
    scene.remove_world_object("xx")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.4         
    p.pose.position.y = -0.2
    p.pose.position.z = 0.205
    scene.add_box("box", p, (0.04, 0.04, 0.04))

    p.pose.position.x = 0.7
    p.pose.position.y = 0
    p.pose.position.z = -0.2065
    scene.add_box("table", p, (0.9, 0.9, 0.775))
    """
    p.pose.position.x = 0.25
    p.pose.position.y = 0
    p.pose.position.z = 0.2
    scene.add_box("xx", p, (0.01, 0.9, 0.04))
    """



    rospy.sleep(1)

    arm_group.set_named_target("home")
    plan1 = arm_group.go()

    # Open the gripper
    hand_group.set_named_target("open")
    plan2 = hand_group.go()



    # put the arm at the 1st grasping position
    pose_target = geometry_msgs.msg.Pose()
    """
    pose_target.orientation.w = 0
    pose_target.orientation.x = -1.57
    pose_target.orientation.y = 0 
    pose_target.orientation.z = -1.57
    """
    downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
    print("downOrientation: ", downOrientation)

    pose_target.orientation.w = 0
    pose_target.orientation.x = 0.7070904
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0.7070904
    pose_target.position.x = 0.25
    pose_target.position.y = -0.2
    pose_target.position.z = 0.3
    arm_group.set_pose_target(pose_target)
    plan1 = arm_group.plan()
    plan1 = arm_group.go()

    pose_target.position.z -= 0.1
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
    arm_group.set_named_target("home")
    plan5 = arm_group.go()

    rospy.spin()
    roscpp_shutdown()