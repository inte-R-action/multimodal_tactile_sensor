/***************************************************************************
  APDS9960 library from SparkFun is used to get raw data
  VL53L1X library from Sparkfun is used to obrain distance data
  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include <Wire.h>
#include <SparkFun_APDS9960.h>

   std_msgs::Float64 distance_msg;
   std_msgs::Float64 gesture_msg;

   ros::Publisher pub_distance("distance", &distance_msg);
   ros::Publisher pub_gesture("gesture", &gesture_msg);
   
 ros::NodeHandle nh;
   
// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();

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
  nh.advertise(pub_gesture);

  //Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  distanceSensor.setDistanceModeLong();

    // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
   // Serial.println(F("APDS-9960 initialization complete"));
  } else {
   // Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) {
    //Serial.println(F("Gesture sensor is now running"));
  } else {
   // Serial.println(F("Something went wrong during gesture sensor init!"));
  }

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

//  float gesturedata= apds.readGesture();
    
    distance_msg.data = distance_data;
  //  gesture_msg.data = gesturedata;
    
    pub_distance.publish(&distance_msg);
  //  pub_gesture.publish(&gesture_msg);
    publisher_timer = millis() + 1; //publish once a second
  }
