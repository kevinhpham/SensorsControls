#include<iostream>>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
using std::cout;

class rando
{
    public:
    const std::string PLANNING_GROUP = "arm";

        rando(){
            cout << "Whats good world";
            ros::AsyncSpinner spinner(1);
            spinner.start();
            moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
            movearm(group);
            ros::waitForShutdown();
        };

        void movearm(moveit::planning_interface::MoveGroupInterface& group){
            cout << "MOVING ARM BOSS" << std::endl;
            group.setRandomTarget();
            group.move();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_inerface_demo");
    rando r; 
}