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
//////VL53L1x Proximity Sensor Library
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
//////APDS-9960 GEsture Sensor Library
#include <Wire.h>
#include <Vector.h>
#include <SparkFun_APDS9960.h>
//////ICM Library
#include <icm20789.h>

   std_msgs::Float32 distance_msg;
   std_msgs::Float32MultiArray array_gesture_msg; 
   
   ros::Publisher pub_distance("distance", &distance_msg);
   ros::Publisher pub_gesture("gesture", &array_gesture_msg);

   std_msgs::Float32MultiArray array_msg_icm;
   ros::Publisher icm("icmData", &array_msg_icm);

   ros::NodeHandle nh;
   float gesturedata[32];

double accelX, accelY, accelZ; //strore the values read from the sensor
double gForceX, gForceY, gForceZ; //use for the calculation of the gforce
double gyroX, gyroY, gyroZ;
float pressure;

// Create ICM20789 object
ICM20789 sensor; 
#define I2C_ADDR 0x68
#define PRE_ADDR 0x63
  
// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();

uint32_t timer;
unsigned long publisher_timer;


SFEVL53L1X distanceSensor;
 
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(57600);
  Wire.begin();
  nh.initNode();
  nh.advertise(pub_distance);
 // nh.advertise(pub_gesture);
  nh.advertise(icm);

/*
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
*/
// allocate memory for ICM20789 message
  array_msg_icm.data = (float*)malloc(sizeof(float) * 7);
  array_msg_icm.data_length = 7;
  for(int i = 0; i < array_msg_icm.data_length; i++)
  array_msg_icm.data[i] = 0.0; 
  sensor.begin(); //first sensor
   recordAccelRegisters();
   recordGyroRegisters();  

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
    {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
     }   
    
publisher_timer = millis() + 10;  

}

// the loop function runs over and over again forever
void loop() {
  if (millis() > publisher_timer) {
  recordAccelRegisters();
  recordGyroRegisters();
  sensor.measure(sensor.ACCURATE);
   rosData();    
  }
  
   nh.spinOnce();
}

void setupICM(){
  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM when AD0 is low
  Wire.write(0x6B); //Accessing the register 6B for Power Management
  Wire.write(0x01); //Setting the sleep register to 0.
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope configuration 
  Wire.write(0x00); //Setting the Gyro to full scale +/- 250 deg
  Wire.endTransmission();
  
  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x1C); //Accessing the register 1C - Accelerometer configuration
  Wire.write(0x00); //Setting the accel to  +/- 2 g
  Wire.endTransmission(); 

  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x37); //Accessing register INT_PIN_CFG
  Wire.write(0x02); //Setting BAYPASS_EN to 1
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x6A); //Accessing register USER_CTRL
  Wire.write(0x00); //Setting I2C_MST_EN (I2C_IF_DIS) to 0
  Wire.endTransmission(); 

 delay(100); // Wait for sensor to stabilize
  }


void recordAccelRegisters(){
  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM when AD0 is low
  Wire.write(0x3B); //Starting register for accel reading
  Wire.endTransmission();    
  Wire.requestFrom(I2C_ADDR,6); //request accel registers, 6 registers from 3B to 40
  while (Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  accelZ = -1 * accelZ;
    }
 

void recordGyroRegisters(){
  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM when AD0 is low
  Wire.write(0x43); //Starting register for gyro reading
  Wire.endTransmission();    
  Wire.requestFrom(I2C_ADDR,6); //request accel registers, 6 registers from 3B to 40
  while (Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into gyroX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into gyroY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into gyroZ
  }

void rosData(){
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }

  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  distance_msg.data = distance;
      
    pressure = sensor.getPressurePa();
    array_msg_icm.data[0] = accelX;
    array_msg_icm.data[1] = accelY;
    array_msg_icm.data[2] = accelZ;
    array_msg_icm.data[3] = gyroX;
    array_msg_icm.data[4] = gyroY;
    array_msg_icm.data[5] = gyroZ;
    array_msg_icm.data[6] = pressure;

  
    
    //Publish distance data
    pub_distance.publish(&distance_msg);
    
    //Publish ICM data 
    icm.publish(&array_msg_icm);    
    

}
