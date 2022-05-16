
#include "ros.h" 
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
ros::NodeHandle nh;

geometry_msgs::Vector3 accVec1;
geometry_msgs::Vector3 gyroVec1;
geometry_msgs::Vector3 accVec2;
geometry_msgs::Vector3 gyroVec2;
geometry_msgs::Vector3 accVec3;
geometry_msgs::Vector3 gyroVec3;

 ros::Publisher pub_acc1("acc1", &accVec1);
 ros::Publisher pub_gyro1("gyro1", &gyroVec1); 
 ros::Publisher pub_acc2("acc2", &accVec2);
 ros::Publisher pub_gyro2("gyro2", &gyroVec2);
 ros::Publisher pub_acc3("acc3", &accVec3);
 ros::Publisher pub_gyro3("gyro3", &gyroVec3);
//ros::Publisher chatter7("chatter7", &str_msg7);

//char hello1[13] = "hello world!";
//char hello2[13] = "hello world!";
//char hello3[13] = "hello world!";
//char hello4[13] = "hello world!";
//char hello5[13] = "hello world!";
//char hello6[13] = "hello world!";
//char hello7[13] = "hello world!";

void setup()
{
  nh.initNode();

  nh.advertise(pub_acc1);
  nh.advertise(pub_gyro1); 
  nh.advertise(pub_acc2);
  nh.advertise(pub_gyro2); 
  nh.advertise(pub_acc3);
  nh.advertise(pub_gyro3);
  
//  nh.advertise(chatter1);
//  nh.advertise(chatter2);
//  nh.advertise(chatter3);
//  nh.advertise(chatter4);
//  nh.advertise(chatter5);
//  nh.advertise(chatter6);
//  nh.advertise(chatter7);
}

void loop()
{
//  str_msg1.data = hello1;
//  str_msg2.data = hello2;
//  str_msg3.data = hello3;
//  str_msg4.data = hello4;
//  str_msg5.data = hello5;
//  str_msg6.data = hello6;
//  str_msg7.data = hello7;
//  
//  chatter1.publish( &str_msg1 );
//  chatter2.publish( &str_msg2 );
//  chatter3.publish( &str_msg3 );
//  chatter4.publish( &str_msg4 );
//  chatter5.publish( &str_msg5 );
//  chatter6.publish( &str_msg6 );
//  chatter7.publish( &str_msg7 );
    pub_acc1.publish(&accVec1);
    pub_gyro1.publish(&gyroVec1);

    pub_acc2.publish(&accVec2);
    pub_gyro2.publish(&gyroVec2);
    
    pub_acc3.publish(&accVec3);
    pub_gyro3.publish(&gyroVec3);
  nh.spinOnce();
  delay(1000);
}
