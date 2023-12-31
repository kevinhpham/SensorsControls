#!/usr/bin/env python

import copy
import actionlib
import rospy
import math
import tf
import roslib
import time
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

# Used to recieve information about the pickup object pose and convert it to other frames
#Currently does not work, need to implement object pose detection system first
class ObjectTransformer(object):

    def __init__(self):
        self.tflistener = tf.TransformListener()
        rospy.loginfo("Waiting for transformlistener...")
        time.sleep(1)

    #Function to convert object pose from camera to base_link of robot
    def poseConverter(self):
        rospy.loginfo("Converting pose of object")
        camera_frame = "head_camera_link"
        object_pose = PoseStamped()
        object_pose.header.frame_id = camera_frame
        object_pose.header.stamp = rospy.Time.now()
        object_pose.pose = Pose(Point(0, 0, 0), Quaternion(0.0, 0.0, 0.0, 1.0)) #currently uses hardcoded value, replace once object detection is working
        while not rospy.is_shutdown():
            try:
                new_pose = self.tflistener.transformPose("base_link",object_pose,)
                break
            except (tf.ConnectivityException , tf.ConnectivityException, tf.ExtrapolationException):
                continue

        rospy.loginfo('Pose of object in base_link frame:')
        rospy.loginfo(new_pose)
        return new_pose


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
    cube_transformer = ObjectTransformer()
    cube_in_grapper = False
    cube_transformer.poseConverter()  #Gets the pose of the object to pick up, currently not functional
    arm_controller.stow()

    head_action.look_at(1.4, 0.0, 0.0, "base_link") #point camera to table
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

