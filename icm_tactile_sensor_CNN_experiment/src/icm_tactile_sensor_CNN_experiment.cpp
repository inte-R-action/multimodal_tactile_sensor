/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 4-March-2020
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

#include <sstream>

#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

using namespace std;

bool ready_to_write = false;
string estimated_area;

ofstream myfilePressure;
ofstream myfileLoadCell;
ofstream myfileTemperature;
ofstream myfileKalmanX;
ofstream myfileKalmanY;
ofstream myfileAccelerometer;
ofstream myfileGyro;
ofstream myfileForceTorque;

bool classification_ready = false;

//ofstream myTextFile;


void pressureCallback(const std_msgs::Float32::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

	if( ready_to_write  == true )
		myfilePressure << msg->data << "\n";
}

//void loadCellCallback(const std_msgs::Float32::ConstPtr& msg)
//{
//	ROS_INFO("Load Cell data received: [%f]", msg->data);
	//if( ready_to_write  == true )
	//	myfileLoadCell << msg->data << "\n";
//}

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
//	ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz);
    	if( ready_to_write  == true )
		myfileForceTorque << msg.Fx << "\t" << msg.Fy << "\t" << msg.Fz << "\t" << msg.Mx<< "\t" << msg.My << "\t" << msg.Mz << "\n";
}

void areaCallback(const std_msgs::String::ConstPtr& msg)
{
		estimated_area = msg->data;
        cout<<"Estimated Area :"<< estimated_area<<endl;
        classification_ready = true;
}


//void ft_robotpos_Callback(const std::string _folderPath)
//{
    //   if( ready_to_write  == true )
  //     myfilerobotposz << target_pose3.position << "\n";
//}


/*
void icmSensorDataCallback(const multimodal_tactile_sensor::icmDataMsg::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%f]", msg->data);

    if( ready_to_write  == true )
        myTextFile << msg->pressure << "\t" << msg->loadCell << "\t" << msg->temperature << "\t" << msg->kalmanX << "\t" << msg->kalmanY << "\t" << msg->kalmanY << "\t" << msg->accelX <<  "\t" << msg->accelY <<  "\t" << msg->accelZ <<  "\t" << msg->gyroX <<  "\t" << msg->gyroY <<  "\t" << msg->gyroZ << "\n";
}
*/

int main(int argc, char** argv)
{

    ros::init(argc, argv, "icm_tactile_sensor_CNN_experiment");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


	ros::Subscriber subPressure = node_handle.subscribe("pressure", 1000, pressureCallback);
	//ros::Subscriber subLoadCell = node_handle.subscribe("load_cell", 1000, loadCellCallback);
	ros::Subscriber subTemperature = node_handle.subscribe("temperature", 1000, temperatureCallback);
	ros::Subscriber subKalmanX = node_handle.subscribe("kalmanX", 1000, kalmanXCallback);
	ros::Subscriber subKalmanY = node_handle.subscribe("kalmanY", 1000, kalmanYCallback);
	ros::Subscriber subAccelerometer = node_handle.subscribe("acc", 1000, accelerometerCallback);
	ros::Subscriber subGyro = node_handle.subscribe("gyro", 1000, gyroCallback);

    ros::Subscriber sub_area = node_handle.subscribe<std_msgs::String>("area", 1000,areaCallback);    

    ros::ServiceClient client = node_handle.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber sub1 = node_handle.subscribe("robotiq_ft_sensor",1000,ft_sensor_Callback);
    
     robotiq_ft_sensor::sensor_accessor srv;    

     //ros::spin();

//    ros::Subscriber subicmSensorData = node_handle.subscribe("icmSensorData", 1000, icmSensorDataCallback);

//	ros::spin();



    // Setup
    // ^^^^^
    //
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_interface:`PlanningSceneInterface`
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
//    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Tactip Grid Exploration - v 0.1.0", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
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
    moveit::planning_interface::MoveGroupInterface::Plan plan;


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
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);


    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the tactile exploration");


    // Definition of parameters for planning Cartesian paths
//    double PLAN_TIME_ = 50.0;
//    bool MOVEIT_REPLAN_ = true;
//    double CART_STEP_SIZE_ = 0.01;

    // The documentation says that 'jump threshold' should be set to 0.0 for simulation and different from 0.0 for the real robot
    // However, it seems that values grater than 0.0 doesn't allow the robot to move
//    double CART_JUMP_THRESH_ = 0.0;
//    bool AVOID_COLLISIONS_ = true;
//    double MAX_VEL_SCALE_FACTOR = 0.3;

//    move_group.setPlanningTime(PLAN_TIME_);
//    move_group.allowReplanning(MOVEIT_REPLAN_);
//    move_group.setMaxVelocityScalingFactor(MAX_VEL_SCALE_FACTOR);


    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot to a 'home' position
    // Begin


    std::map<std::string, double> targetJoints;
    targetJoints["shoulder_pan_joint"] = -2.94*3.1416/180;
    targetJoints["shoulder_lift_joint"] = -13.86*3.1416/180;
    targetJoints["elbow_joint"] = -72.03*3.1416/180;
    targetJoints["wrist_1_joint"] = -94.57*3.1416/180;
    targetJoints["wrist_2_joint"] = 2.79*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180; 

    move_group.setJointValueTarget(targetJoints);



//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualise the planned robot movement");

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot movement");

    // IMPORTANT!!!!!     
    // Use always the 'execute' method to move the robot in the simulation environment and the real robot!
    // The current version of this program still does NOT work properly with the 'move' method
    // Using the 'move' method in this program might generate unexected and dangerous robot movements
    move_group.execute(plan);
    //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

    sleep(3);

//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with the next action");

    // End
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot to a pre-start position
    // Begin

    targetJoints.clear();

    
//  for (int m = 0; m < 10; m++){
    while(1)
    {
    if( classification_ready == true )
    {

    if (estimated_area == "8"){

    targetJoints["shoulder_pan_joint"] = 121.04*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -48.59*3.1416/180;
    targetJoints["elbow_joint"] = -2.18*3.1416/180;
    targetJoints["wrist_1_joint"] = -130.89*3.1416/180;
    targetJoints["wrist_2_joint"] = -114.18*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;
     }
     else if (estimated_area == "7"){
    targetJoints["shoulder_pan_joint"] = 83.04*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -48.46*3.1416/180;
    targetJoints["elbow_joint"] = -2.17*3.1416/180;
    targetJoints["wrist_1_joint"] = -130.60*3.1416/180;
    targetJoints["wrist_2_joint"] = -80.62*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else if (estimated_area == "6"){
    targetJoints["shoulder_pan_joint"] = 46.29*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -48.47*3.1416/180;
    targetJoints["elbow_joint"] = -2.18*3.1416/180;
    targetJoints["wrist_1_joint"] = -130.60*3.1416/180;
    targetJoints["wrist_2_joint"] = -46.31*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else if (estimated_area == "5"){
    targetJoints["shoulder_pan_joint"] = 112.72*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -23.98*3.1416/180;
    targetJoints["elbow_joint"] = -2.06*3.1416/180;
    targetJoints["wrist_1_joint"] = -155.71*3.1416/180;
    targetJoints["wrist_2_joint"] = -108.02*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else if (estimated_area == "4"){
    targetJoints["shoulder_pan_joint"] = 86.02*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -26.33*3.1416/180;
    targetJoints["elbow_joint"] = -2.14*3.1416/180;
    targetJoints["wrist_1_joint"] = -150.18*3.1416/180;
    targetJoints["wrist_2_joint"] = -86.31*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else if (estimated_area == "3"){
    targetJoints["shoulder_pan_joint"] = 59.52*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -26.32*3.1416/180;
    targetJoints["elbow_joint"] = -2.15*3.1416/180;
    targetJoints["wrist_1_joint"] = -149.39*3.1416/180;
    targetJoints["wrist_2_joint"] = -58.55*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else if (estimated_area == "2"){
    targetJoints["shoulder_pan_joint"] = 108.02*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -17.40*3.1416/180;
    targetJoints["elbow_joint"] = 26.02*3.1416/180;
    targetJoints["wrist_1_joint"] = -189.18*3.1416/180;
    targetJoints["wrist_2_joint"] = -105.74*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else if (estimated_area == "1"){
    targetJoints["shoulder_pan_joint"] = 85.26*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -32.26*3.1416/180;
    targetJoints["elbow_joint"] = 55.74*3.1416/180;
    targetJoints["wrist_1_joint"] = -205.47*3.1416/180;
    targetJoints["wrist_2_joint"] = -85.84*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else if (estimated_area == "0"){
    targetJoints["shoulder_pan_joint"] = 58.97*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -30.79*3.1416/180;
    targetJoints["elbow_joint"] = 50.20*3.1416/180;
    targetJoints["wrist_1_joint"] = -200.33*3.1416/180;
    targetJoints["wrist_2_joint"] = -56.83*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;        
    }
     else {
    targetJoints["shoulder_pan_joint"] = -2.94*3.1416/180;
    targetJoints["shoulder_lift_joint"] = -13.86*3.1416/180;
    targetJoints["elbow_joint"] = -72.03*3.1416/180;
    targetJoints["wrist_1_joint"] = -94.57*3.1416/180;
    targetJoints["wrist_2_joint"] = 2.79*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.81*3.1416/180;     
    }
     
    estimated_area = "";
    classification_ready = false;


    // touch on the PCB test
    move_group.setJointValueTarget(targetJoints);

//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualise the planned robot movement");

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing PRE-START position plan (%.2f%% acheived)",success * 100.0);

//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot movement");

    // IMPORTANT!!!!!     
    // Use always the 'execute' method to move the robot in the simulation environment and the real robot!
    // The current version of this program still does NOT work properly with the 'move' method
    // Using the 'move' method in this program might generate unexected and dangerous robot movements
    move_group.execute(plan);
    //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with the next action");

    // End
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    sleep(3);
    }   // if 
    } //for loop
    ros::shutdown();

    return 0;
}
