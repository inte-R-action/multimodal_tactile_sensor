/*
 *********************************************************************************
 * Author: Gorkem Anil AL
 * Email: gorkem.a6l@bath.ac.uk
 * Date: 4-March-2020
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description: Two different velocity is used, high velocity is set to 0.1, the low velocity is set to 0.02
 *
 *********************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "multimodal_tactile_sensor/icmDataMsg.h"
#include <fstream>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>

#include <ur_controllers/speed_scaling_state_controller.h>
#include "ur_msgs/SetSpeedSliderFraction.h" //type of the service for speed slider

#include <ur_robot_driver/hardware_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

//Force-torque sensor libraries
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <sstream>

using namespace std;
string objectString = "";
bool robotMove = false;
string gripper_state = "";
namespace rvt = rviz_visual_tools;
int robot_execute_code;
bool ready_to_write = false;
float distance;


ofstream myfilePressure;
ofstream myfileLoadCell;
ofstream myfileTemperature;
ofstream myfileKalmanX;
ofstream myfileKalmanY;
ofstream myfileAccelerometer;
ofstream myfileGyro;
ofstream myfileForceTorque;
float proximity;
float velocity_constant; 
//ofstream myTextFile;


void pressureCallback(const std_msgs::Float32::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

	if( ready_to_write  == true )
		myfilePressure << msg->data << "\n";
}

void loadCellCallback(const std_msgs::Float32::ConstPtr& msg)
{
//	ROS_INFO("Load Cell data received: [%f]", msg->data);
	if( ready_to_write  == true )
		myfileLoadCell << msg->data << "\n";
}

void temperatureCallback(const std_msgs::Float32::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

	if( ready_to_write  == true )
		myfileTemperature << msg->data << "\n";
}

void kalmanXCallback(const std_msgs::Float32::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

	if( ready_to_write  == true )
		myfileKalmanX << msg->data << "\n";
}

void kalmanYCallback(const std_msgs::Float32::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

	if( ready_to_write  == true )
		myfileKalmanY << msg->data << "\n";
}

void accelerometerCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

	if( ready_to_write  == true )
		myfileAccelerometer << msg->x << "\t" << msg->y << "\t" << msg->z << "\n";
}

void gyroCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

	if( ready_to_write  == true )
		myfileGyro << msg->x << "\t" << msg->y << "\t" << msg->z << "\n";
}

void ft_sensor_Callback(const robotiq_ft_sensor::ft_sensor& msg)
{
	//ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz);
	if( ready_to_write  == true )
		myfileForceTorque << msg.Fx << "\t" << msg.Fy << "\t" << msg.Fz << "\n";
	
}


void DistanceValueCallback(const std_msgs::Float64::ConstPtr& msg)
{
		proximity = msg->data;
  // ROS_INFO("I heard: [%f]", proximity);

       if (proximity < 300.0){
           velocity_constant = 0.20;  
        }
        else {
           velocity_constant = 0.25;  
        }

}

void robotMoveCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data);
 
 	//objectString.clear();

    objectString = msg->data; 

    //cout << "Robot move: " << robotMove << endl; 
    //cout << "Object: " << objectString << endl;
}

void robotExecuteCallback(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr& msg)
{
    robot_execute_code = msg->result.error_code.val;
}
struct jnt_angs{double angles[6];};
std::map<std::string, jnt_angs> create_joint_pos(){
    std::map<std::string, jnt_angs> joint_positions;
    joint_positions["home"] = {-2.54, -76.32, 75, -92, -90.0, 0.0}; //position: x = -0.709, y = -0.011, z = 0.315
    joint_positions["initial_pos"] = {53.78, -88.5, 87.37, -91.13, -88.7, 0.0}; 
    joint_positions["bring_side_1"] = {53.78, -88.5, 87.37, -91.13, -88.7, 0.0}; //position: x = -0.190, y = -0.488, z = 0.499
    return joint_positions;
};

class moveit_robot {
    public:
        // Setup
        // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
        // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
        // are used interchangably.
        const std::string PLANNING_GROUP;

        // The :move_group_interface:`MoveGroup` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface move_group;

        // We will use the :planning_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const robot_state::JointModelGroup* joint_model_group;

        // Now, we call the planner to compute the plan and visualize it.
        // The plan variable contains the movements that the robot will perform to move
        // from one point to another
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        moveit::core::RobotStatePtr current_state;
        std::vector<double> joint_group_positions;

        // Visualization
        // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
        // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
        moveit_visual_tools::MoveItVisualTools visual_tools;
        Eigen::Isometry3d text_pose;

        // Joint positions map
        std::map<std::string, jnt_angs> joint_positions;

        // Gripper message, cmd publisher and status subscriber
       // std_msgs::String  gripper_msg;
      //  ros::Publisher gripper_cmds_pub;
      //  ros::Subscriber gripper_feedback_sub;

        ros::Subscriber robot_execute_sub;

        std_msgs::String  robot_status_msg;
        ros::Publisher robot_status_pub;

        ros::ServiceServer set_speed_slider_srv_; //new

        // Force sensor
        ros::ServiceClient ft_client;
        ros::Subscriber ft_sub1;
        robotiq_ft_sensor::sensor_accessor ft_srv;

        moveit_robot(ros::NodeHandle* node_handle);
        void move_robot(std::map<std::string, double> targetJoints, std::string robot_action, std::string jnt_pos_name);
       // void open_gripper();
       // void close_gripper();
        void move_cartesian(double dist1, double dist2, double dist3);
        bool plan_to_pose(geometry_msgs::Pose pose);
        geometry_msgs::Pose transform_pose(geometry_msgs::Pose input_pose);
    
    private:
        ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

};

moveit_robot::moveit_robot(ros::NodeHandle* node_handle) : nh_(*node_handle), PLANNING_GROUP("manipulator"), visual_tools("world"), move_group(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP)) {

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //Eigen::Isometry3d 
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "HRI Static Demo - v 0.1.0", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // 
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("UR3 robot", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("UR3 robot", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


    // Now, we call the planner to compute the plan and visualize it.
    // The plan variable contains the movements that the robot will perform to move
    // from one point to another
    //moveit::planning_interface::MoveGroupInterface::Plan plan;

    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
  //  move_group.setMaxVelocityScalingFactor(0.25);
  //  move_group.setMaxAccelerationScalingFactor(0.15);
 


  //  ur_controllers::SpeedScalingStateController moveit_robot;    
    


    joint_positions = create_joint_pos();

   // gripper_cmds_pub = nh_.advertise<std_msgs::String>("UR2Gripper", 1);
   // gripper_feedback_sub = nh_.subscribe("Gripper2UR", 1, gripperStatusCallback);
    robot_status_pub = nh_.advertise<std_msgs::String>("RobotStatus", 10);

    robot_execute_sub = nh_.subscribe("execute_trajectory/result", 1, robotExecuteCallback);

   // ft_client = nh_.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
   // ft_sub1 = nh_.subscribe("robotiq_ft_sensor",100,ftSensorCallback);

    // Add collision objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^

    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    moveit_msgs::PlanningScene planning_scene;
    //planning_scene.robot_state.attached_collision_objects.clear();
    //planning_scene.world.collision_objects.clear();
    //planning_scene_diff_publisher.publish(planning_scene);
    std::vector<std::string> object_ids;
   // object_ids.push_back("gripper");
   // object_ids.push_back("camera");
    object_ids.push_back("ground");
    planning_scene_interface.removeCollisionObjects(object_ids);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    //geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();


    //gripper_object.touch_links = std::vector<std::string>{ "ee_link"};
    
    //planning_scene.world.collision_objects.push_back(gripper_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);


    //camera_object.touch_links = std::vector<std::string>{ "ee_link", "gripper"};
    
    //planning_scene.world.collision_objects.push_back(camera_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);

    /* First, define the REMOVE object message*/
    //moveit_msgs::CollisionObject remove_camera;
    //remove_camera.id = "camera";
    //remove_camera.header.frame_id = "base_link";
    //remove_camera.operation = remove_camera.REMOVE;

    /* Carry out the REMOVE + ATTACH operation */
    ROS_INFO("Attaching the camera to the robot");
    //planning_scene.world.collision_objects.clear();
    //planning_scene.world.collision_objects.push_back(remove_camera);
   // planning_scene.robot_state.attached_collision_objects.push_back(camera_object);
    planning_scene_diff_publisher.publish(planning_scene);
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    //collision_objects.push_back(camera_object);

    //ROS_INFO_NAMED("tutorial", "Attach the camera to the robot");
    //move_group.attachObject(camera_object.id);

    // Add ground plane object to robot
    moveit_msgs::CollisionObject ground_object;
    ground_object.header.frame_id = move_group.getPlanningFrame();
    ground_object.id = "ground";
    shape_msgs::SolidPrimitive ground_primitive;
    ground_primitive.type = ground_primitive.BOX;
    ground_primitive.dimensions.resize(3);
    ground_primitive.dimensions[0] = 2;
    ground_primitive.dimensions[1] = 2;
    ground_primitive.dimensions[2] = 0.01;

    geometry_msgs::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = -ground_primitive.dimensions[2];

    ground_object.primitives.push_back(ground_primitive);
    ground_object.primitive_poses.push_back(ground_pose);
    ground_object.operation = ground_object.ADD;

    collision_objects.push_back(ground_object);

    ROS_INFO_NAMED("tutorial", "Add ground into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
}

void moveit_robot::move_robot(std::map<std::string, double> targetJoints, std::string robot_action, std::string jnt_pos_name){
    
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    targetJoints.clear();
    targetJoints["shoulder_pan_joint"] = joint_positions[jnt_pos_name].angles[0]*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = joint_positions[jnt_pos_name].angles[1]*3.1416/180;
    targetJoints["elbow_joint"] = joint_positions[jnt_pos_name].angles[2]*3.1416/180;
    targetJoints["wrist_1_joint"] = joint_positions[jnt_pos_name].angles[3]*3.1416/180;
    targetJoints["wrist_2_joint"] = joint_positions[jnt_pos_name].angles[4]*3.1416/180;
    targetJoints["wrist_3_joint"] = joint_positions[jnt_pos_name].angles[5]*3.1416/180;

    robot_status_msg.data = robot_action;
    robot_status_pub.publish(robot_status_msg);

    move_group.setStartState(*move_group.getCurrentState());

    move_group.setJointValueTarget(targetJoints);

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing new move position plan (%.2f%% acheived)",success * 100.0);

    robot_execute_code = 0;
    move_group.asyncExecute(plan);
    while(){
        
    }
   // move_group.execute(plan);

}

void moveit_robot::move_cartesian(double dist1, double dist2, double dist3){

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    move_group.setStartState(*move_group.getCurrentState());

    geometry_msgs::Pose target_home = move_group.getCurrentPose().pose;

  //  cout << "Pose:" << move_group.getCurrentPose().pose.position << endl;
    // Vector to store the waypoints for the planning process
    std::vector<geometry_msgs::Pose> waypoints;
    // Stores the first target pose or waypoint
    geometry_msgs::Pose target_pose3 = target_home;
    
    target_pose3.position.x = target_pose3.position.x + dist1;    
    target_pose3.position.y = target_pose3.position.y + dist2;
    target_pose3.position.z = target_pose3.position.z + dist3;

    waypoints.push_back(target_pose3);
    int num_waypoints = plan.trajectory_.joint_trajectory.points.size();  
    cout << "Number of way points :" << num_waypoints << endl;

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);

    plan.trajectory_ = trajectory;

    const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names;    //gets the names of the joints being updated in the trajectory

    kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(0).positions);
   // geometry_msgs::end_effector = Pose;
    const std::string end_effector = move_group.getEndEffectorLink().c_str();

    //cout << "End-effector:" << end_effector << endl;
    
    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;

   // cout << "current_end_effector_state:" << getGlobalLinkTransform(end_effector)<< endl;

    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
    trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;

     

    for(int i = 0; i < num_waypoints - 1; i++)      //loop through all waypoints
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
        next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
         
        //set joints for next waypoint
        kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);
 
        //do forward kinematics to get cartesian positions of end effector for next waypoint
        next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
 
        //get euclidean distance between the two waypoints
        euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) + 
                            pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) + 
                            pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2), 0.5);
 
        new_timestamp = curr_waypoint->time_from_start.toSec() + (euclidean_distance / 0.05 * velocity_constant);      //start by printing out all 3 of these!
        old_timestamp = next_waypoint->time_from_start.toSec();

        cout << "Velocity:" << (0.1 * velocity_constant) <<endl;   // print used velocity 
        //update next waypoint timestamp & joint velocities/accelerations if joint velocity/acceleration constraints allow
        if(new_timestamp > old_timestamp)
            next_waypoint->time_from_start.fromSec(new_timestamp);
        else
        {
            //ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");
        }
         
        //update current_end_effector_state for next iteration
        current_end_effector_state = next_end_effector_state;
    }
     
    //now that timestamps are updated, update joint velocities/accelerations (used updateTrajectory from iterative_time_parameterization as a reference)
    for(int i = 0; i < num_waypoints; i++)
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);            //set current, previous & next waypoints
        if(i > 0)
            prev_waypoint = &plan.trajectory_.joint_trajectory.points.at(i-1);
        if(i < num_waypoints-1)
            next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
 
        if(i == 0)          //update dt's based on waypoint (do this outside of loop to save time)
            dt1 = dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        else if(i < num_waypoints-1)
        {
            dt1 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
            dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        }
        else
            dt1 = dt2 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
 
        for(int j = 0; j < joint_names.size(); j++)     //loop through all joints in waypoint
        {
            if(i == 0)                      //first point
            {
                q1 = next_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
            else if(i < num_waypoints-1)    //middle points
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = next_waypoint->positions.at(j);
            }
            else                            //last point
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
 
            if(dt1 == 0.0 || dt2 == 0.0)
                v1 = v2 = a = 0.0;
            else
            {
                v1 = (q2 - q1)/dt1;
                v2 = (q3 - q2)/dt2;
                a = 2.0*(v2 - v1)/(dt1+dt2);
            }
 
            //actually set the velocity and acceleration
            curr_waypoint->velocities.at(j) = (v1+v2)/2;
            curr_waypoint->accelerations.at(j) = a;
            cout << "curr_waypoint_velocities:" << (v1+v2)/2 << endl; // print velocity valuesc

        }
    }
        
     move_group.asyncExecute(plan);
        
}

void pick_up_object(moveit_robot &Robot, double down_move_dist1, double down_move_dist2, double down_move_dist3)
{
    // Robot moves down, grasps part and moves back to original position
    Robot.move_cartesian(down_move_dist1,down_move_dist2,down_move_dist3);
   
 //   Robot.move_cartesian(-down_move_dist1,-down_move_dist2,down_move_dist3, 0.03);
  //  sleep(1.5);
   // Robot.move_cartesian(down_move_dist1,down_move_dist2,down_move_dist3, 0.2);
}

void home(std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    string bring_cmd = "home";

    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);
}

void initial_pos(std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    string bring_cmd = "initial_pos";

    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);
 
}

void take_side(string bring_cmd, std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    // Move to position above side
    //Robot.move_robot(targetJoints, bring_cmd, bring_cmd);

    // Move down, pick side up, move up
    pick_up_object(Robot, -0.0310, 0.0130, 0);
   // pick_up_object(Robot, -0.0410, 0.0210, 0);
   // pick_up_object(Robot, -0.039, 0.0250, 0);
   // pick_up_object(Robot, -0.036, 0.0280, 0);
    pick_up_object(Robot, -0.034, 0.0310, 0);
   // pick_up_object(Robot, -0.031, 0.0330, 0);
   // pick_up_object(Robot, -0.027, 0.0370, 0);
   // pick_up_object(Robot, -0.025, 0.0380, 0);
   // pick_up_object(Robot, -0.021, 0.0410, 0);
    //pick_up_object(Robot, -0.018, 0.0420, 0);
  //  pick_up_object(Robot, -0.030, 0.0440, 0);

    // Move to user delivery position
   // Robot.move_robot(targetJoints, bring_cmd, string("deliver_2_user"));

    // Move down, set down side, move up
  //  set_down_object(Robot, 0.03, 0.5);

    // Return to home position
    //home(targetJoints, Robot);
}

int main(int argc, char** argv)
{

    string frame_id = "velocity_control";
    ros::init(argc, argv, frame_id);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
 

    // High level move commands subscriber
	ros::Subscriber subRobotPosition = node_handle.subscribe("RobotMove", 1000, robotMoveCallback);
    ros::Subscriber subproximity = node_handle.subscribe("distance", 1000, DistanceValueCallback);
    // Object recognition subscriber
  //  ros::Subscriber subObjectRecog = node_handle.subscribe("ObjectStates", 1000, objectDetectionCallback);

    ros::ServiceClient client  = node_handle.serviceClient<ur_msgs::SetSpeedSliderFraction>("/ur_hardware_interface/set_speed_slider"); //robot velocity control
    ur_msgs::SetSpeedSliderFraction set_speed_frac;

       if (proximity < 300){
            set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity
            client.call(set_speed_frac);       
             cout << "velocity constant:" << velocity_constant << endl;    
        }
        else {
         set_speed_frac.request.speed_slider_fraction = 0.25; // change velocity
        client.call(set_speed_frac);   
         cout << "velocity constant:" << velocity_constant << endl;
          // velocity_constant = 0.25;  
        }

    // Robot object
    moveit_robot Robot(&node_handle);
    // Map to hold values to send to robot
    std::map<std::string, double> targetJoints;
    
    string last_obj_string = "";

    // Send robot to home position
    home(targetJoints, Robot);

    // Send Robot to starting position
    initial_pos(targetJoints, Robot);
    // wait position
    Robot.robot_status_msg.data = "Done";
    Robot.robot_status_pub.publish(Robot.robot_status_msg);
    cout << ">>>>-- Waiting for command --<<<<" << endl;


   // while( ros::ok() )
   // {
            targetJoints.clear();

    //take_side("bring_side_1", targetJoints, Robot);
    cout << "velocity constant:" << velocity_constant << endl;
    home(targetJoints, Robot);
    ros::shutdown();

    return 0;
}
