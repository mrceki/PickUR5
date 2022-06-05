#! /usr/bin/env python
import sys
import rospy
import rospy
import moveit_commander
import geometry_msgs.msg


class Sim():
    def init(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick2", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        arm_group = moveit_commander.MoveGroupCommander("ur5_arm")
        hand_group = moveit_commander.MoveGroupCommander("gripper")

        self.box_name = "box"
        self.table_name = "table"
        self.robot = robot
        self.scene = scene
        self.arm_group = arm_group
        self.hand_group = hand_group
        self.add_block()
        self.add_table()

    def add_block(self):

        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.Pose()
        box_pose.orientation.w = 0
        box_pose.position.x = 0.6
        box_pose.position.y = -0.2
        box_pose.position.z = 0,773999
        box_name = "box"
        scene.add_block(box_name, box_pose, size=(0.04, 0.04, 0.04))

        self.box_name = box_name

    def add_table(self):

        table_name = self.table_name
        scene = self.scene

        table_pose = geometry_msgs.msg.Pose()
        table_pose.position.x = 0.8
        table_pose.position.y = 0.0
        table_pose.position.z = 0.181
        table_name = "table"
        scene.add_table(table_name, table_pose, size=(0.8, 0.8, 0.01))
        self.table_name = table_name
    
    def move(self):
        arm_group = self.arm_group
        hand_group = self.hand_group

        arm_group.set_named_target("home")
        plan1 = hand_group.go()
        # Open the gripper
        hand_group.set_named_target("open")
        plan2 = hand_group.go()

        # 1st grasping position
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

        # 3rd grasping position
        pose_target.position.z = 1.5
        arm_group.set_pose_target(pose_target)
        plan4 = arm_group.plan()
        plan4 = arm_group.go()

def main():
    try:
        x = Sim()
        x.add_block()
        x.add_table()
        x.move()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
