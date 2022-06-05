#! /usr/bin/env python

import rospy

from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown

from actionlib import SimpleActionClient, GoalStatus

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import PickupAction, PickupGoal
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_msgs.msg import MoveItErrorCodes

from tf.transformations import quaternion_from_euler

import sys
import copy
import os

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


class Pick_Place:
    
    def __init__(self):
        # Retrieve params:
        self._table_object_name = rospy.get_param('~table_object_name', 'Grasp_Table')
        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'Grasp_Object')

        self._grasp_object_width = rospy.get_param('~grasp_object_width', 0.01)

        self._arm_group     = rospy.get_param('~ur5_arm', 'ur5_arm')
        self._gripper_group = rospy.get_param('~gripper', 'gripper')

        self._approach_retreat_desired_dist = rospy.get_param('~approach_retreat_desired_dist', 0.2)
        self._approach_retreat_min_dist = rospy.get_param('~approach_retreat_min_dist', 0.1)

        # Create (debugging) publishers:
        self._grasps_pub = rospy.Publisher('grasps', PoseArray, queue_size=1, latch=True)
        self._places_pub = rospy.Publisher('places', PoseArray, queue_size=1, latch=True)

        # Create planning scene and robot commander:
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()

        rospy.sleep(1.0)

        # Clean the scene:
        self._scene.remove_world_object(self._table_object_name)
        self._scene.remove_world_object(self._grasp_object_name)

        # Add table and Coke can objects to the planning scene:
        self._pose_table    = self._add_table(self._table_object_name)
        self._pose_box= self._add_grasp_block_(self._grasp_object_name)

        rospy.sleep(1.0)

        # Define target place pose:
        self._pose_place = Pose()

        self._pose_place.position.x = self._pose_box.position.x
        self._pose_place.position.y = self._pose_box.position.y - 0.20
        self._pose_place.position.z = self._pose_box.position.z
        self._pose_place.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))

        self._pose_box.position.z = self._pose_box.position.z - 0.9 # base_link is 0.9 above !!!!!!!!!!!!!!!!!!!!!
        self._pose_place.position.z = self._pose_place.position.z + 0.05 # target pose a little above !!!!!!!!!!!!

        # Retrieve groups (arm and gripper):
        self._arm     = self._robot.get_group(self._arm_group)
        self._gripper = self._robot.get_group(self._gripper_group)

        self._arm.set_named_target('pickup')# !!!!!!!Kontrol edilecek
        self._arm.go(wait=True)
        print("Pickup pose")


    def __del__(self):
        # Clean the scene:
        self._scene.remove_world_object(self._grasp_object_name)
        self._scene.remove_world_object(self._table_object_name)

    def _add_table(self, name):
        """
        Create and add table to the scene
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.8
        p.pose.position.y = 0.0
        p.pose.position.z = 0.181

        # Table size from ~/.gazebo/models/table/model.sdf, using the values
        # for the surface link.
        self._scene.add_box(name, p, (0.8, 0.8, 0.01))

        return p.pose

    def _add_grasp_block_(self, name):
        """
        Create and add block to the scene
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.6
        p.pose.position.y = -0.2
        p.pose.position.z = 0,773999
        self._scene.add_box(name, p, (0.05, 0.05, 0.05))

        return p.pose

    def _generate_places(self, target):
        """
        Generate places (place locations), based on
        https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/
        baxter_pick_place/src/block_pick_place.cpp
        Create a Pose Array data for the pose of the place
        """

        # Generate places:

    def _create_pickup_goal(self, group, target, grasps):
        """
        Create a MoveIt! PickupGoal
        Create a goal for picking up the grasping object
        """

        # Create goal:
        goal = PickupGoal()

        goal.group_name  = group
        goal.target_name = target

        goal.possible_grasps.extend(grasps)

        goal.allowed_touch_objects.append(target)

        goal.support_surface_name = self._table_object_name

        # Configure goal planning options:
        goal.allowed_planning_time = 7.0

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 20

        return goal

    def _pickup(self, group, target, width):
        """
        Pick up a target using the planning group
        """

        # Obtain possible grasps from the grasp generator server:
        grasps = self._generate_grasps(self._pose_box, width)

        # Create and send Pickup goal:
        goal = self._create_pickup_goal(group, target, grasps)

        state = self._pickup_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Pick up goal failed!: %s' % self._pickup_ac.get_goal_status_text())
            return None

        result = self._pickup_ac.get_result()


        return True

        """def _place(self, group, target, place):
        """
       # Place a target using the planning group
        """

        # Obtain possible places:
        places = self._generate_places(place)

        # Create and send Place goal:
        goal = self._create_place_goal(group, target, places)

        state = self._place_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Place goal failed!: %s' % self._place_ac.get_goal_status_text())
            return None

        result = self._place_ac.get_result()

        # Check for error:
        err = result.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot place target %s!: %s' % (group, target, str(moveit_error_dict[err])))

            return False

        return True
        """
    def _publish_grasps(self, grasps):

        """
        Publish grasps as poses, using a PoseArray message
        """ 

        if self._grasps_pub.get_num_connections() > 0:
            msg = PoseArray()
            msg.header.frame_id = self._robot.get_planning_frame()
            msg.header.stamp = rospy.Time.now()

            for grasp in grasps:
                p = grasp.grasp_pose.pose

                msg.poses.append(Pose(p.position, p.orientation))

            self._grasps_pub.publish(msg)

    def _publish_places(self, places):
        """
        Publish places as poses, using a PoseArray message
        """

        if self._places_pub.get_num_connections() > 0:
            msg = PoseArray()
            msg.header.frame_id = self._robot.get_planning_frame()
            msg.header.stamp = rospy.Time.now()

            for place in places:
                msg.poses.append(place.place_pose.pose)

            self._places_pub.publish(msg)


def main():
    p = Pick_Place()

    rospy.spin()


if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place')

    """
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/table.urdf -urdf -x 0.85 -y 0.0 -z 0.73 -model my_object
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/block.urdf -urdf -x 0.5 -y -0.0 -z 0.77 -model block
    """
    """
    table_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'table.urdf'
    table_name = 'table'
    table_pose = Pose(position=Point(x=0.85, y=0.0, z=0.70+0.03)) # increase z by 0.03 to make gripper reach block

    block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'block.urdf'
    block_name = 'block'
    block_pose = Pose(position=Point(x=0.5, y=0.0, z=0.74+0.03))
    """
    main()

    roscpp_shutdown()
    