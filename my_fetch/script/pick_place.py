#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson
# Author: Di Sun

import copy
import actionlib
import rospy
import math
from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction,GripperCommandGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal,Object
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, Grasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

#controlls closing and opening gripper
class GripperClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        rospy.loginfo("Waiting for gripper_controller...")
        self.client.wait_for_server()

    def closeGrip(self):                            #Send command to robot gripper to close hand
        goal = GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 70.0
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def openGrip(self):                            #Send command to robot gripper to open hand
        goal = GripperCommandGoal()
        goal.command.position = 0.1
        self.client.send_goal(goal)
        self.client.wait_for_result()

#moves arm to poses inorder to pick up cube
class ArmController(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")
        self.arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    def startPose(self):
        joints = self.arm_joints
        pose = [1.57, 0, 0, -1.57, 0.0, 0, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
            
    def ApproachPose(self):
        joints = self.arm_joints
        pose = [-.384, -0.593, 1.87, 1.22, -1.13, 1.95, 0.99]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
            
    def pickupPose(self):
        joints = self.arm_joints
        pose = (-0.38, -0.45, 1.8, 1.27, -1.169, 1.9, 0.977)
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return   

    def approachPlace(self):
        gripper_frame = 'gripper_link'
        gripper_pose = PoseStamped()
        gripper_pose.header.frame_id ='base_link'
        gripper_pose.header.stamp = rospy.Time.now()
        gripper_pose.pose = Pose(Point(0.809, 0.2, 0.75), Quaternion(0.01, 0.0, 0.0, 1.0))
        while not rospy.is_shutdown():
            result = self.move_group.moveToPose(gripper_pose, gripper_frame,0.05)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return   

    def tuck(self):
        joints = self.arm_joints
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def stow(self):
        joints = self.arm_joints
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    #move_base = MoveBaseClient()
    #torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    gripper_client = GripperClient()
    arm_controller = ArmController()
    cube_in_grapper = False
    arm_controller.stow()

    head_action.look_at(1.4, 0.0, 0.0, "base_link")
    gripper_client.openGrip()
    arm_controller.startPose()
    arm_controller.ApproachPose()
    arm_controller.pickupPose()
    gripper_client.closeGrip()
    arm_controller.ApproachPose()
    head_action.look_at(1.4, 0.0, 0.3, "base_link")
    arm_controller.startPose()
    arm_controller.approachPlace()
    gripper_client.openGrip()
    arm_controller.stow()
    rospy.loginfo("Demo Complete!")

