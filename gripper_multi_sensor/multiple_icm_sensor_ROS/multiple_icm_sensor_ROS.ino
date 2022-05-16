/******************************************************************
  ICP10xx library is used for the TDK InvenSense ICP-101xx
 series of barometric pressure sensors.

 Kalman filter was implemented to use roll and pitch values. 
 This code is ready to test with UR3 robot.
 sensor2 is used now

 Multimodal tactile finger human to robot handover action program
 Program was modified on February-2022
******************************************************************/
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
//#include <sensor_msgs/Imu.h>


#include<Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
//#include <icp101xx.h> 
#include <icm20789.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf



std_msgs::Float32MultiArray array_msg_icm;
ros::Publisher first_icm("icmData1", &array_msg_icm);

std_msgs::Float32MultiArray array_msg_icm2;
ros::Publisher second_icm("icmData2", &array_msg_icm2);

std_msgs::Float32MultiArray array_msg_icm3;
ros::Publisher third_icm("icmData3", &array_msg_icm3);

ros::NodeHandle nh;



double accelX, accelY, accelZ; //strore the values read from the sensor
double gForceX, gForceY, gForceZ; //use for the calculation of the gforce

double gyroX, gyroY, gyroZ;
double rotX, rotY, rotZ;

int16_t accelCount[3]; 
int16_t gyroCount[3]; 
int16_t accelCount2[3]; 
int16_t gyroCount2[3]; 
int16_t accelCount3[3]; 
int16_t gyroCount3[3]; 

double ax1, ay1, az1;       // Stores the real accel value in g's
double ax2, ay2, az2;       // Stores the real accel value in g's
double ax3, ay3, az3;       // Stores the real accel value in g's

double gx1, gy1, gz1;       // Stores the real gyro value 
double gx2, gy2, gz2;       // Stores the real gyro value 
double gx3, gy3, gz3;       // Stores the real gyro value 

float pressure1, pressure2, pressure3;

float base_val1 = 0.0, base_val_kalmanx1, base_val_kalmany1;
float base_val2 = 0.0, base_val_kalmanx2, base_val_kalmany2;
float base_val3 = 0.0, base_val_kalmanx3, base_val_kalmany3;

float Last_val1[2], Last_val_kalmanx1[2], Last_val_kalmany1[2];
float Last_val2[2], Last_val_kalmanx2[2], Last_val_kalmany2[2];
float Last_val3[2], Last_val_kalmanx3[2], Last_val_kalmany3[2];

long publisher_timer;
int counter = 0;

uint32_t timer;


int temp_2;

// Sensor is an ICM20789 object
ICM20789 sensor; 
ICM20789 sensor2; 
ICM20789 sensor3; 

#define I2C_ADDR 0x68
#define PRE_ADDR 0x63
#define TCAADDR 0x70 // for I2C multiplexer

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup() {
  Wire.begin();
  //Serial.begin(57600);  
  nh.getHardware()->setBaud(57600);
  nh.initNode(); 
          
  tcaselect(0);
  // allocate memory for icm message
  array_msg_icm.data = (float*)malloc(sizeof(float) * 7);
  array_msg_icm.data_length = 7;

  for(int i = 0; i < array_msg_icm.data_length; i++)
    array_msg_icm.data[i] = 0.0; 

   sensor.begin(); //first sensor
   recordAccelRegisters();
   recordGyroRegisters();  
   nh.advertise(first_icm);
   
  tcaselect(1);
    // allocate memory for icm message
  array_msg_icm2.data = (float*)malloc(sizeof(float) * 7);
  array_msg_icm2.data_length = 7;

  for(int i = 0; i < array_msg_icm2.data_length; i++)
    array_msg_icm2.data[i] = 0.0; 
   sensor2.begin(); //second sensor  
   recordAccelRegisters();
   recordGyroRegisters();     
    nh.advertise(second_icm);
    
  tcaselect(2);
  // allocate memory for icm message
  array_msg_icm3.data = (float*)malloc(sizeof(float) * 7);
  array_msg_icm3.data_length = 7;

  for(int i = 0; i < array_msg_icm3.data_length; i++)
    array_msg_icm3.data[i] = 0.0; 
   sensor3.begin(); //third sensor
   recordAccelRegisters();
   recordGyroRegisters();     
   nh.advertise(third_icm);
}

void loop() {
  if (millis() > publisher_timer) {
  tcaselect(0);
   
     sensor.measure(sensor.ACCURATE);
     recordAccelRegisters();
     recordGyroRegisters();

 /*        
    ax1 = accelX; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
    ay1 = accelY;
    az1 = accelZ;

    gx1 = gyroX; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
    gy1 = gyroY;
    gz1 = gyroZ;
    delay(10);
  */   
      pressure1 = sensor.getPressurePa();
      array_msg_icm.data[0] = accelX;
      array_msg_icm.data[1] = accelY;
      array_msg_icm.data[2] = accelZ;
      array_msg_icm.data[3] = gyroX;
      array_msg_icm.data[4] = gyroY;
      array_msg_icm.data[5] = gyroZ;
      array_msg_icm.data[6] = pressure1; 
      
      first_icm.publish(&array_msg_icm);
      
//    pressure = sensor.getPressurePa();
//    kalmanxy1.x = kalAngleX;
//    kalmanxy1.y = kalAngleY;
//    kalmanxy1.z = pressure;
//    pub_kalman1.publish(&kalmanxy1);
    
    //Serial.print(kalAngleX); Serial.print("\t"); 
   // Serial.print(kalAngleY); Serial.print("\t"); 

//   sensor.getAccelData(accelCount); //get raw x,y,z accelerometer data
//   sensor.getGyroData(gyroCount); //get raw x,y,z gyro data
    

  tcaselect(1);
  sensor2.measure(sensor2.ACCURATE);
     recordAccelRegisters();
     recordGyroRegisters();

      pressure2 = sensor2.getPressurePa();
      array_msg_icm2.data[0] = accelX;
      array_msg_icm2.data[1] = accelY;
      array_msg_icm2.data[2] = accelZ;
      array_msg_icm2.data[3] = gyroX;
      array_msg_icm2.data[4] = gyroY;
      array_msg_icm2.data[5] = gyroZ;
      array_msg_icm2.data[6] = pressure2;
      
      second_icm.publish(&array_msg_icm2);

//    pressure2 = sensor2.getPressurePa();
//    kalmanxy2.x = kalAngleX2;
//    kalmanxy2.y = kalAngleY2;
//    kalmanxy2.z = pressure2;    
//    pub_kalman2.publish(&kalmanxy2); 
  /*
    ax2 = accelX; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
    ay2 = accelY;
    az2 = accelZ;

    gx2 = gyroX; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
    gy2 = gyroY;
    gz2 = gyroZ;
*/

//  sensor2.getAccelData(accelCount2); //get raw x,y,z accelerometer data
//  sensor2.getGyroData(gyroCount2); //get raw x,y,z gyro data
//    ax2 = (float)accelCount2[0]; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
//    ay2 = (float)accelCount2[1];
//    az2 = (float)accelCount2[2];
//
//    gx2 = (float)gyroCount2[0]; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
//    gy2 = (float)gyroCount2[1];
//    gz2 = (float)gyroCount2[2];
// Serial.print("second sensor"); Serial.print("\t"); Serial.print(sensor2.getPressurePa()/1); Serial.print("\t");
 //Serial.println(sensor2.getTemperatureC());

 
  //delay(10);
  
 tcaselect(2);
  sensor3.measure(sensor3.ACCURATE);
     recordAccelRegisters();
     recordGyroRegisters();  

     pressure3 = sensor3.getPressurePa();
      array_msg_icm3.data[0] = accelX;
      array_msg_icm3.data[1] = accelY;
      array_msg_icm3.data[2] = accelZ;
      array_msg_icm3.data[3] = gyroX;
      array_msg_icm3.data[4] = gyroY;
      array_msg_icm3.data[5] = gyroZ;
      array_msg_icm3.data[6] = pressure3;
      
      third_icm.publish(&array_msg_icm3); 
      
     /*
    ax3 = accelX; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
    ay3 = accelY;
    az3 = accelZ;

    gx3 = gyroX; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
    gy3 = gyroY;
    gz3 = gyroZ;
*/

//  sensor3.getAccelData(accelCount3); //get raw x,y,z accelerometer data
//  sensor3.getGyroData(gyroCount3); //get raw x,y,z gyro data
//    ax3 = (float)accelCount3[0]; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
//    ay3 = (float)accelCount3[1];
//    az3 = (float)accelCount3[2];
//
//    gx3 = (float)gyroCount3[0]; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
//    gy3 = (float)gyroCount3[1];
//    gz3 = (float)gyroCount3[2];
// Serial.print("third sensor"); Serial.print("\t"); Serial.println(sensor3.getPressurePa()/1); //Serial.print("\t");
// Serial.println(gz3);
  //rosData();
 //delay(10);

 publisher_timer = millis() + 20; 
 
  }
  
  nh.spinOnce();  
  delay(10);
}

void rosData() {

 //   counter++;
    publisher_timer = millis() + 1; //publish once a second
    
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
 // processAccelData();   
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
 // processGyroData(); 
  }
