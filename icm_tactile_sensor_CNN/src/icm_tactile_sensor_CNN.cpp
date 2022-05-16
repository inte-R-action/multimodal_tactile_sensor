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


ofstream myfilePressure;
ofstream myfileLoadCell;
ofstream myfileTemperature;
ofstream myfileKalmanX;
ofstream myfileKalmanY;
ofstream myfileAccelerometer;
ofstream myfileGyro;
ofstream myfileForceTorque;


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

   if( argc < 5 )
   {
        std::cout << "ERROR: missing input parameters" << std::endl;

        cout << "Expecting input parameters" << endl;
        cout << "1: folder path (string)" << endl;
        cout << "2: experiment number (string)" << endl;
        cout << "3: number of rows (int)" << endl;
        cout << "4: number of columns (int)" << endl;
        cout << "5: touch increment value (float)" << endl;
   }
    else
    {

    	// List of input parametera
    std::string folderPath = argv[1];			// folder to store the data
    std::string expNumber = argv[2];			// experiment number
    int numOfRows = std::stoi(argv[3]);			// number of rows (large side)
    int numOfColumns = std::stoi(argv[4]); 		// number of columns (short side)
    float touchInc = std::stof(argv[5])/1000;			// touch increment value divided by 1000 to scale it to millimeters

    //float touchInc = 0;

    ros::init(argc, argv, "icm_tactile_sensor_CNN");
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
    
    ros::ServiceClient client = node_handle.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber sub1 = node_handle.subscribe("robotiq_ft_sensor",1000,ft_sensor_Callback);
    
     robotiq_ft_sensor::sensor_accessor srv;    

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
    targetJoints["shoulder_pan_joint"] = -0.74*3.1416/180;
    targetJoints["shoulder_lift_joint"] = -27.84*3.1416/180;
    targetJoints["elbow_joint"] = 38.3*3.1416/180;
    targetJoints["wrist_1_joint"] = -101.58*3.1416/180;
    targetJoints["wrist_2_joint"] = -89.71*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.26*3.1416/180; 

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


    // home touch position (converts input from degrees to radians)
  //  targetJoints["shoulder_pan_joint"] = -9.31*3.1416/180;	// (deg*PI/180)
   // targetJoints["shoulder_lift_joint"] = -25.13*3.1416/180;
   // targetJoints["elbow_joint"] = 44.10*3.1416/180;
  //  targetJoints["wrist_1_joint"] = -108.33*3.1416/180;
  //  targetJoints["wrist_2_joint"] = -90.09*3.1416/180;
  //  targetJoints["wrist_3_joint"] = 0.04*3.1416/180;

    targetJoints["shoulder_pan_joint"] = -0.87*3.1416/180;	// (deg*PI/180)
    targetJoints["shoulder_lift_joint"] = -25.53*3.1416/180;
    targetJoints["elbow_joint"] = 40.70*3.1416/180;
    targetJoints["wrist_1_joint"] = -106.36*3.1416/180;
    targetJoints["wrist_2_joint"] = -89.72*3.1416/180;
    targetJoints["wrist_3_joint"] = 0.28*3.1416/180;

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

    sleep(1);



    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot to the start position for the exploration of the grid
    // Begin

//    robot_state::RobotStatePtr kinematic_state = robot_state::RobotStatePtr(move_group.getCurrentState());

    // Gets the current pose of the robot
    geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;


    // Predefined distances in meters to move the robot
    double UP_MOVE = 0.1;    // 0.1m or 10cm 
    double DOWN_MOVE = 0.1;
    double LEFT_MOVE = 0.02; // 0.02m or 2cm
    double RIGHT_MOVE = 0.02;
    double BACKWARD_MOVE = 0.02;
    double FORWARD_MOVE = 0.02;


    // Prints initial pose (position and orientation) of the end-effector
    std::cout << "Pose" << std::endl;
    std::cout << "Position [x, y, z]: " << target_pose1.position.x << ", " << target_pose1.position.y << ", " << target_pose1.position.z << std::endl;
    std::cout << "Orientation [x, y, z, w]: " << target_pose1.orientation.x << ", " << target_pose1.orientation.y << ", " << target_pose1.orientation.z << ", " << target_pose1.orientation.w << std::endl;


    sleep(3);


    move_group.setPlanningTime(10.0);

    moveit::core::RobotState start_state(*move_group.getCurrentState());

    geometry_msgs::Pose start_pose2;

    start_state.setFromIK(joint_model_group, start_pose2);

    move_group.setStartState(start_state);


    // When done with the path constraint be sure to clear it.
    move_group.clearPathConstraints();

    float robotStep = 0.001;	// 1mm
    float robotStepX = 0.0010;	// 1mm
//    float robotTouchZ = 0.015 + touchInc;	// 10mm it was 16mm (0.016)
    float robotTouchZ = 0.0 + touchInc;	// 10mm it was 16mm (0.016)

    // touch on the silicon
    float xPosition [3] = { 0.0, 0.0, -robotStepX};
    float yPosition [3] = { 0.0, 0.0,   0.0};
    float zPosition [3] = {-robotTouchZ, robotTouchZ,   0.0};


    // touch on the PCB
    //float yHomePosition [numOfColumns] = {0.0, -0.001, -0.002, -0.003, -0.004, -0.005, -0.006, -0.007, -0.008, -0.009, -0.010, -0.011, -0.012, -0.013};

	float yHomePosition	= 0.0;

    move_group.setStartState(*move_group.getCurrentState());
    // Gets the current pose of the robot
    geometry_msgs::Pose target_home = move_group.getCurrentPose().pose;

    geometry_msgs::Pose homeZPosition = move_group.getCurrentPose().pose;

    for( int k = 0; k < numOfColumns; k++ )
    {
          // set force and torque values of the FT sensor 0   
       //   srv.request.command_id = srv.request.COMMAND_SET_ZERO; 
          //  if(client.call(srv)){
		//	ROS_INFO("ret: %s", srv.response.res.c_str());
		//}

        move_group.setStartState(*move_group.getCurrentState());

        // Vector to store the waypoints for the planning process
        std::vector<geometry_msgs::Pose> waypoints;
        // Stores the first target pose or waypoint
        geometry_msgs::Pose target_pose3 = target_home;
        // Decrements current X position by BACKWARD_MOVE*3
//        target_pose3.position.y = target_pose3.position.y - yHomePosition[k];
        target_pose3.position.y = target_pose3.position.y - yHomePosition;
        yHomePosition = yHomePosition - robotStep - 0.0015; // Move the robot 3mm on y axis, robotStep is 1mm + 2 mm 
        waypoints.push_back(target_pose3);

        // We want the Cartesian path to be interpolated at a resolution of 1 cm
        // which is why we will specify 0.01 as the max step in Cartesian
        // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
        // Warning - disabling the jump threshold while operating real hardware can cause
        // large unpredictable motions of redundant joints and could be a safety issue
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

        // Visualize the plan in RViz
        visual_tools.deleteAllMarkers();
        visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
        for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
        visual_tools.trigger();

        // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
        // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
        // the trajectory manually, as described [here](https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4).
        // Pull requests are welcome.

        // You can execute a trajectory like this.
        move_group.execute(trajectory);

    //    sleep(1);

//        std::cout << "DEBUG -- 2" << std::endl;

        for( int j = 0; j < numOfRows; j++)
        {
                    // set force and torque values of the FT sensor 0   
          srv.request.command_id = srv.request.COMMAND_SET_ZERO; 
            if(client.call(srv)){
			ROS_INFO("ret: %s", srv.response.res.c_str());
		            }
//			myfile.open (folderPath + "/icmSensorData_exp_" + expNumber + "r" + j + "c" + k + ".txt");
			myfilePressure.open (folderPath + "/icm_pressure_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
 			myfileTemperature.open (folderPath + "/icm_temperature_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
			//myfileLoadCell.open (folderPath + "/icm_loadcell_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
			myfileKalmanX.open (folderPath + "/icm_kalmanx_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
			myfileKalmanY.open (folderPath + "/icm_kalmany_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
			myfileAccelerometer.open (folderPath + "/icm_accel_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
			myfileGyro.open (folderPath + "/icm_gyro_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
            myfileForceTorque.open (folderPath + "/icm_ft_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
 //           myfilerobotposz.open (folderPath + "/icm_robotpos_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");
            
//            myTextFile.open (folderPath + "/icm_sensor_data_exp_" + expNumber + "r" + std::to_string(j) + "c" + std::to_string(k) + ".txt");

//            if (myTextFile.is_open())
			if (myfilePressure.is_open() && myfileKalmanX.is_open() && myfileKalmanX.is_open() && myfileAccelerometer.is_open() && myfileGyro.is_open() && myfileTemperature.is_open() && myfileForceTorque.is_open()) //&& myfileLoadCell.is_open() )
				ready_to_write = true;
			else
				cout << "Unable to open file";  	


            for( int i = 0; i < 3; i++ )
            {

            move_group.setStartState(*move_group.getCurrentState());
            // Gets the current pose of the robot
            geometry_msgs::Pose target_poseInit = move_group.getCurrentPose().pose;
            // Vector to store the waypoints for the planning process
            std::vector<geometry_msgs::Pose> waypoints;
            // Stores the first target pose or waypoint
            geometry_msgs::Pose target_pose3 = target_poseInit;
            // Decrements current X position by BACKWARD_MOVE*3
//            std::cout << "[X, Y, Z] = " << xPosition[i] << ", " << yPosition[i] << ", " << zPosition[i] << std::endl;
// Save robot position on z axis to observe the robot pos in each tap. 3. position in the file is the position where robot applies force to the tactile sensor
          //  myfilerobotposz << -(target_pose3.position.z * 1000.0) - 188.18 << "\n";

            if( i == 0 )
            	target_pose3.position.z = homeZPosition.position.z;

            target_pose3.position.x = target_pose3.position.x + xPosition[i];
            target_pose3.position.y = target_pose3.position.y + yPosition[i];
            target_pose3.position.z = target_pose3.position.z + zPosition[i];
            waypoints.push_back(target_pose3);
            
            //std::cout << "Position [z]: " << target_pose3.position.z << std::endl;     
            // Save robot position on z axis to observe the robot pos in each tap. 3. position in the file is the position where robot applies force to the tactile sensor
           // myfilerobotposz << -(target_pose3.position.z * 1000.0) - 192.089 << "\n";
          

            // We want the Cartesian path to be interpolated at a resolution of 1 cm
            // which is why we will specify 0.01 as the max step in Cartesian
            // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause
            // large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            // Visualize the plan in RViz
            visual_tools.deleteAllMarkers();
            visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
            for (std::size_t i = 0; i < waypoints.size(); ++i)
            visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
            visual_tools.trigger();

            // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
            // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
            // the trajectory manually, as described [here](https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4).
            // Pull requests are welcome.

            // You can execute a trajectory like this.
            move_group.execute(trajectory);

            if ( i == 2 )
                sleep(0.5);
            else
                sleep(2.0);

//            std::cout << "DEBUG -- 1" << std::endl;

            }

//            if (myTextFile.is_open() )
            if (myfilePressure.is_open() &&  myfileKalmanX.is_open() && myfileKalmanX.is_open() && myfileAccelerometer.is_open() && myfileGyro.is_open() && myfileTemperature.is_open() && myfileForceTorque.is_open())// && myfileLoadCell.is_open())
			{
	   			ready_to_write = false;
				myfilePressure.close();
				myfileTemperature.close();
				//myfileLoadCell.close();
				myfileKalmanX.close();
				myfileKalmanY.close();
				myfileAccelerometer.close();
				myfileGyro.close();
                myfileForceTorque.close();
             //   myfilerobotposz.close();
			}
			else
				cout << "Unable to close file. File was not open";

        }
    }

    ros::shutdown();

}

    return 0;
}
