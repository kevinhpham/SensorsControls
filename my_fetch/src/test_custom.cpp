#include<iostream>>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>

class control{
    public:
    const std::string PLANNING_GROUP = "arm";

        control(int argc, char **argv){
            ros::init(argc, argv, "move_group_interface_demo");
            ros::NodeHandle obj;
            ros::AsyncSpinner spinner(1);
            spinner.start();
            moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            ros::Publisher display_publisher = obj.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
            ROS_INFO("REFERENCE FRAME: %s", group.getPlanningFrame().c_str());
            ROS_INFO("REFERENCE FRAME: %s", group.getEndEffectorLink().c_str());
            movearm(group);
            moveJoints(group);

            sleep(2.0);
        };

    void movearm(moveit::planning_interface::MoveGroupInterface& group)
    {
        //Moves end effector to specific pose
        moveit_msgs::DisplayTrajectory display_trajectory;   
        geometry_msgs::Pose targetPose;
        targetPose.position.x = 0.5;
        targetPose.position.y = 0.5;
        targetPose.position.z = 0.5;

        targetPose.orientation.w = 0;
        targetPose.orientation.x = 0;
        targetPose.orientation.y = 0;
        targetPose.orientation.z = 0;
        // plane movement before executing
        group.setPoseTarget(targetPose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        group.move();
    };

    void moveJoints(moveit::planning_interface::MoveGroupInterface& group)
    {
        moveit::core::RobotStatePtr current_state = group.getCurrentState(); // Retrieve current state of robot.
        const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //Create pointer to the joint_model_group for the arm
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions); //copy the current joint positions of the joint_model_group of arm into joint_group_positions
        joint_group_positions[0] = -1.0;  // move base joint of arm -1 radians
        group.setJointValueTarget(joint_group_positions); // Update targets joint positions
        moveit::planning_interface::MoveGroupInterface::Plan my_plan; //call planner interface to plan out path and visualise it
        bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); //check if plan was successful or not
        ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED"); // print to console
    };

};




int main(int argc, char **argv)
{
    control r = control(argc,argv);
}
