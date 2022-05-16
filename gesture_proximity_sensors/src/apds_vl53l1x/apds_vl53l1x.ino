/***************************************************************************
  APDS9960 library from SparkFun is used to get raw data
  VL53L1X library from Sparkfun is used to obrain distance data
  
 ***************************************************************************/
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include <Wire.h>
#include <Vector.h>
#include <SparkFun_APDS9960.h>

   std_msgs::Float32 distance_msg;
   std_msgs::Float32MultiArray array_gesture_msg; 
   
   ros::Publisher pub_distance("distance", &distance_msg);
   ros::Publisher pub_gesture("gesture", &array_gesture_msg);
   
   ros::NodeHandle nh;
   float gesturedata[32];
   
// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();

uint32_t timer;
unsigned long publisher_timer;

typedef Vector<int> Elements;

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(57600);
  Wire.begin();
  nh.initNode();
  //nh.advertise(pub_distance);
  nh.advertise(pub_gesture);
  
/*

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  distanceSensor.setDistanceModeLong();
*/


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
  
     // allocate memory for gesture message
  array_gesture_msg.data = (float*)malloc(sizeof(float) * 32);
  array_gesture_msg.data_length = 32;

  for(int i = 0; i < array_gesture_msg.data_length; i++)
    array_gesture_msg.data[i] = 0.0; 

publisher_timer = millis() + 10;  

}

// the loop function runs over and over again forever
void loop() {

      rosData();     
      nh.spinOnce();
}

void rosData(){
  
   if( millis() > publisher_timer )
  {
    /*
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  float distance_data = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
*/
  // int storage_array[32];
  // Elements vector;
   //vector.setStorage(storage_array);
   
   //int gesturedata= apds.readGesture(true);
    
   // distance_msg.data = distance_data;
 //   for (int k=0; k <=31; k++)
   // array_gesture_msg.data[k] = apds.readGesture(true);
    apds.readGesture(true, array_gesture_msg.data);
        for( int i=0; i<32; i++)
      {
        Serial.print(array_gesture_msg.data[i]); 
        Serial.print(" ");
      }
        Serial.println();
   // vector.push_back(gesturedata);

   
    pub_gesture.publish(&array_gesture_msg);  
    
    //array_gesture_msg.data = 0;
   // pub_distance.publish(&distance_msg);
    
    
    publisher_timer = millis() + 10; //publish once a second
  }
}
