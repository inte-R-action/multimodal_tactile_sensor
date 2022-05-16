#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>
#include <map>
#include <sstream>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <ros/ros.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "multimodal_tactile_sensor/icmDataMsg.h"
#include <ur_controllers/speed_scaling_state_controller.h>
//#include "multimodal_tactile_sensor/SetSpeedSliderFraction.h"
#include "ur_msgs/SetSpeedSliderFraction.h" //type of the service for speed slider

#include <ur_robot_driver/hardware_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

using namespace std;

string objectString = "";
bool robotMove = false;
string gripper_state = "";
namespace rvt = rviz_visual_tools;
int robot_execute_code;
double ft_readings [6];
float proximity;
float distance;
float velocity_constant; 


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


int main(int argc, char** argv)
{

    // Set up ROS stuff
    string frame_id = "speed_control";
    ros::init(argc, argv, frame_id);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
 
    // High level move commands subscriber
    ros::Subscriber subproximity = node_handle.subscribe("distance", 1000, DistanceValueCallback);
    ros::Publisher velocity_change = node_handle.advertise<std_msgs::Float64>("/ur_hardware_interface/set_speed_slider",1000);
   // ros::Publisher chatter_pub = node_handle.advertise<std_msgs::String>("chatter", 1000);
   
    ros::ServiceClient client  = node_handle.serviceClient<ur_msgs::SetSpeedSliderFraction>("/ur_hardware_interface/set_speed_slider");

    ur_msgs::SetSpeedSliderFraction set_speed_frac;

   // set_speed_frac.request.speed_slider_fraction = 0.25; // change velocity
/*
if (client.call(set_speed_frac))
{
  ROS_INFO("Successfull, velocity fraction was changed.");
}
else
{
  ROS_ERROR("Failed to call service.");
}

  ROS_INFO("Speed slider fraction: %f", set_speed_frac.request.speed_slider_fraction);

*/

/*
   if (client.call(req, res))
     {
       ROS_INFO("Speed slider fraction: %f", req.speed_slider_fraction);
       ROS_ERROR("Failed to call service");
     }
     else
     {
       ROS_ERROR("Failed to call service");
       return 1;
     }
*/

/*
       if (velocity_constant = 0.10){
         set_speed_frac.request.speed_slider_fraction = 0.10; // change velocity
            client.call(set_speed_frac);       
        }
        else {
         set_speed_frac.request.speed_slider_fraction = 0.25; // change velocity
        client.call(set_speed_frac);   
          // velocity_constant = 0.25;  
        }

    cout << "velocity constant:" << velocity_constant << endl;
*/
//int count = 0;
  // while( count < 3 )

while( ros::ok() )
 {


/*
ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
  [&](const ros::TimerEvent&) {
    
  //  ROS_INFO("Velocity Constant %f", setSpeedSlider);
    std_msgs::Float64 msg;   
    msg.data = velocity_constant;
   // ROS_INFO("Velocity Constant %f", msg.data);
    velocity_change.publish(msg);  
    chatter_count++;
  });
  */ 

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

   // cout << "velocity constant:" << velocity_constant << endl;


    //cout << "velocity constant:" << velocity_constant << endl;

     //count = count + 1;
}

    ros::shutdown();
    return 0;
}
