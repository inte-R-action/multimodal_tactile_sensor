/***************************************************************************
  APDS9960 library from SparkFun is used to get raw data
  VL53L1X library from Sparkfun is used to obrain distance data
  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <ros.h>
#include <std_msgs/Float32.h>

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include <Wire.h>
#include <SparkFun_APDS9960.h>

   std_msgs::Float32 distance_msg; // create message for the distance measured by proximity sensor

   ros::Publisher pub_distance("distance", &distance_msg); // publish distance message from proximity sensor 
   ros::NodeHandle nh;


uint32_t timer;
long publisher_timer;

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(57600);
  Wire.begin();
  nh.initNode();
  nh.advertise(pub_distance);

  //Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  distanceSensor.setDistanceModeLong();

}

// the loop function runs over and over again forever
void loop() {

      rosData();
      
nh.spinOnce();
}

void rosData(){
  
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  float distance_data = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
     
    distance_msg.data = distance_data;
    
    pub_distance.publish(&distance_msg);
    publisher_timer = millis() + 1; //publish once a second
  }
