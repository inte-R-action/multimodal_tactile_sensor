/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 6-March-2020
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description:
 *
 *********************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>


int readyToMoveRobot = 0;
std::string movementStr = " ";


// Callback that receives the gesture recognised (String type): up, down, left, right
void gestureCallback(const std_msgs::String::ConstPtr& msg)
{
    if( readyToMoveRobot == 0 )
    {
        movementStr = " ";
        readyToMoveRobot = 1;
        movementStr = msg->data.c_str();
        ROS_INFO("I received: [%s]", msg->data.c_str());
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_hand_gesture_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Create ROS subscriber to get data from gesture recognition
    ros::Subscriber gestureSub = node_handle.subscribe("gesture", 1, gestureCallback);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
 //   Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    move_group.setPoseReferenceFrame("world");

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    ros::Rate loop_rate(10);
    ros::spinOnce();

    std::map<std::string, double> targetHomeJoints;

    std::string upStr = "up";
    std::string downStr = "down";
    std::string leftStr = "left";
    std::string rightStr = "right";
    std::string noStr = "No movement";

    while( ros::ok() )
    {
        if( readyToMoveRobot == 1 )
        {
            if( upStr.compare(movementStr) == 0 )
            {
                targetHomeJoints["shoulder_pan_joint"] = -1.58;
                targetHomeJoints["shoulder_lift_joint"] = -1.58;
                targetHomeJoints["elbow_joint"] = 0.50;
                targetHomeJoints["wrist_1_joint"] = -1.58;
                targetHomeJoints["wrist_2_joint"] = 0.0;
                targetHomeJoints["wrist_3_joint"] = 0.0;  
                ROS_INFO("Moving the robot: [%s]", upStr.c_str());
            }
            else if( downStr.compare(movementStr) == 0 )
            {
                targetHomeJoints["shoulder_pan_joint"] = -1.58;
                targetHomeJoints["shoulder_lift_joint"] = -0.95;
                targetHomeJoints["elbow_joint"] = 1.69;
                targetHomeJoints["wrist_1_joint"] = -2.33;
                targetHomeJoints["wrist_2_joint"] = 0.0;
                targetHomeJoints["wrist_3_joint"] = 0.0;  
                ROS_INFO("Moving the robot: [%s]", downStr.c_str());
            }
            else if( leftStr.compare(movementStr) == 0 )
            {
                targetHomeJoints["shoulder_pan_joint"] = -3.12;
                targetHomeJoints["shoulder_lift_joint"] = -1.33;
                targetHomeJoints["elbow_joint"] = 1.19;
                targetHomeJoints["wrist_1_joint"] = -2.33;
                targetHomeJoints["wrist_2_joint"] = 0.0;
                targetHomeJoints["wrist_3_joint"] = 0.0;  
                ROS_INFO("Moving the robot: [%s]", leftStr.c_str());
            }
            else if( rightStr.compare(movementStr) == 0 )
            {
                targetHomeJoints["shoulder_pan_joint"] = 0.0;
                targetHomeJoints["shoulder_lift_joint"] = -1.33;
                targetHomeJoints["elbow_joint"] = 1.19;
                targetHomeJoints["wrist_1_joint"] = -2.33;
                targetHomeJoints["wrist_2_joint"] = 0.0;
                targetHomeJoints["wrist_3_joint"] = 0.0;  
                ROS_INFO("Moving the robot: [%s]", rightStr.c_str());
            }
            else
            {
                ROS_INFO("Moving the robot: [%s]", noStr.c_str());
                // do not do any movement
            }

            move_group.setJointValueTarget(targetHomeJoints);

            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//            sleep(2);
//            usleep(500000);

            // execute plan
            move_group.execute(my_plan);
            // move_group.move();

            readyToMoveRobot = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
