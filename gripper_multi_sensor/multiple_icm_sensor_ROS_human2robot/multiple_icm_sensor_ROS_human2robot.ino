/******************************************************************
  ICP10xx library is used for the TDK InvenSense ICP-101xx
 series of barometric pressure sensors.

 Kalman filter was implemented to use roll and pitch values. 
 This code is ready to test with UR3 robot.
 sensor2 is used now

 Multimodal tactile finger human to robot handover action program
******************************************************************/
#include <ros.h>
#include <std_msgs/Float32.h>
//#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
//#include <sensor_msgs/Imu.h>

#include<Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
//#include <icp101xx.h> 
#include <icm20789.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf


geometry_msgs::Vector3 sensor_pressure_msg; // put the reading of three pressure data into one message as vector
geometry_msgs::Vector3 accVec1;
geometry_msgs::Vector3 gyroVec1;
geometry_msgs::Vector3 accVec2;
geometry_msgs::Vector3 gyroVec2;
geometry_msgs::Vector3 accVec3;
geometry_msgs::Vector3 gyroVec3;

//std_msgs::Float32 load_cell_msg;

//std_msgs::Float32MultiArray sensor_data;
std_msgs::Bool touch_msg;
std_msgs::Bool touch2_msg;
//std_msgs::Bool release_msg;
//sensor_msgs::Imu imu_msg;

 //ros::Publisher pub_pressure("pressure", &sensor_pressure_msg);
 ros::Publisher pub_touch("touch", &touch_msg); // first touch to feel the object
 ros::Publisher pub_touch2("touch2", &touch2_msg); //second touch to grasp the object
// ros::Publisher pub_release("release_item", &release_msg);
 
// ros::Publisher pub_acc1("acc1", &accVec1);
 ros::Publisher pub_gyro1("gyro1", &gyroVec1); 
// ros::Publisher pub_acc2("acc2", &accVec2);
 ros::Publisher pub_gyro2("gyro2", &gyroVec2);
// ros::Publisher pub_acc3("acc3", &accVec3);
 //ros::Publisher pub_gyro3("gyro3", &gyroVec3);
 
 //ros::Publisher chatter("chatter", &imu_msg);
  // ros::Publisher chatter("chatter", &sensor_data);
   
  ros::NodeHandle nh;
 //ros::NodeHandle_<ArduinoHardware, 3 ,3, 512, 512> nh; 


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

double pressure1, pressure2, pressure3;

float base_val1 = 0.0, base_val_kalmanx1, base_val_kalmany1;
float base_val2 = 0.0, base_val_kalmanx2, base_val_kalmany2;
float base_val3 = 0.0, base_val_kalmanx3, base_val_kalmany3;

float Last_val1[2], Last_val_kalmanx1[2], Last_val_kalmany1[2];
float Last_val2[2], Last_val_kalmanx2[2], Last_val_kalmany2[2];
float Last_val3[2], Last_val_kalmanx3[2], Last_val_kalmany3[2];

long publisher_timer;
int counter = 0;

uint32_t timer;

long int pressure;
bool touch = false, touch2 = false, ref_val=false, release_item = false;

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
  Serial.begin(57600);  
  //nh.getHardware()->setBaud(115200);
  nh.initNode(); 
  
 //  nh.advertise(pub_pressure);
  //nh.advertise(pub_touch);
 // nh.advertise(pub_acc1);
  nh.advertise(pub_gyro1); 
 // nh.advertise(pub_acc2);
  nh.advertise(pub_gyro2); 
//  nh.advertise(pub_acc3);
 // nh.advertise(pub_gyro3);
   
 // nh.advertise(chatter); 

// nh.advertise(pub_kalman1);
// nh.advertise(pub_kalman2);
// nh.advertise(pub_kalman3);
 nh.advertise(pub_touch);
  nh.advertise(pub_touch2);
// nh.advertise(pub_release);
    
  tcaselect(0);
   sensor.begin(); //first sensor
   recordAccelRegisters();
   recordGyroRegisters();  
  tcaselect(1);
   sensor2.begin(); //second sensor  
   recordAccelRegisters();
   recordGyroRegisters();     
  tcaselect(2);
   sensor3.begin(); //third sensor
   recordAccelRegisters();
   recordGyroRegisters();     
}

void loop() {
  if (millis() > publisher_timer) {
  tcaselect(0);

//################## FORCE SENSOR #####################################
    // Optional: Measurement mode
    //    sensor.FAST: ~3ms
    //    sensor.NORMAL: ~7ms (default)
    //    sensor.ACCURATE: ~24ms
    //    sensor.VERY_ACCURATE: ~95ms
   // sensor.measure(sensor.FAST);
   
   sensor.measure(sensor.ACCURATE);
     recordAccelRegisters();
     recordGyroRegisters();

//    pressure = sensor.getPressurePa();
//    kalmanxy1.x = kalAngleX;
//    kalmanxy1.y = kalAngleY;
//    kalmanxy1.z = pressure;
//    pub_kalman1.publish(&kalmanxy1);
    
    //Serial.print(kalAngleX); Serial.print("\t"); 
   // Serial.print(kalAngleY); Serial.print("\t"); 
    
    ax1 = accelX; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
    ay1 = accelY;
    az1 = accelZ;

    gx1 = gyroX; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
    gy1 = gyroY;
    gz1 = gyroZ;
    delay(10);
//   sensor.getAccelData(accelCount); //get raw x,y,z accelerometer data
//   sensor.getGyroData(gyroCount); //get raw x,y,z gyro data
//    ax1 = (float)accelCount[0]; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
//    ay1 = (float)accelCount[1];
//    az1 = (float)accelCount[2];
//
//    gx1 = (float)gyroCount[0]; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
//    gy1 = (float)gyroCount[1];
//    gz1 = (float)gyroCount[2];
    
  // Serial.print("first sensor"); Serial.print("\t"); Serial.println(sensor.getPressurePa()/1); //Serial.print("\t"); 
   // Serial.println(gx1); //Serial.print("\t");

  tcaselect(1);
  sensor2.measure(sensor2.ACCURATE);
     recordAccelRegisters();
     recordGyroRegisters();

//    pressure2 = sensor2.getPressurePa();
//    kalmanxy2.x = kalAngleX2;
//    kalmanxy2.y = kalAngleY2;
//    kalmanxy2.z = pressure2;    
//    pub_kalman2.publish(&kalmanxy2); 
    
//Serial.print(kalAngleX2); Serial.print("\t"); 
//Serial.print(kalAngleY2); Serial.print("\t"); 

    ax2 = accelX; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
    ay2 = accelY;
    az2 = accelZ;

    gx2 = gyroX; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
    gy2 = gyroY;
    gz2 = gyroZ;

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
  delay(10);
 tcaselect(2);
  sensor3.measure(sensor3.ACCURATE);
     recordAccelRegisters();
     recordGyroRegisters();  

     
    ax3 = accelX; // get actual g value, this depends on scale being set, sensitivity scale factor 16384.0
    ay3 = accelY;
    az3 = accelZ;

    gx3 = gyroX; // get actual g value, this depends on scale being set, sensitivity scale factor 131.0
    gy3 = gyroY;
    gz3 = gyroZ;

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
  rosData();
 //delay(10);
  }
  nh.spinOnce();  
  delay(10);
}

void rosData() {
    ///////////////////publish three pressure data////////////////////////////    
    //sensor_pressure_msg.x= sensor.getPressurePa(); 
   // sensor_pressure_msg.y = sensor2.getPressurePa();
    //sensor_pressure_msg.z = sensor3.getPressurePa();
    
//////////Gyro and accelerometer message for the first sensor///////////////////////
    accVec1.x = ax1;
    accVec1.y = ay1;
    accVec1.z = az1;

    gyroVec1.x = gx1;
    gyroVec1.y = gy1;
    gyroVec1.z = gz1;

///////Gyro and accelerometer message for the second sensor////////////////
    accVec2.x = ax2;
    accVec2.y = ay2;
    accVec2.z = az2;

    gyroVec2.x = gx2;
    gyroVec2.y = gy2;
    gyroVec2.z = gz2;
    
////////Gyro and accelerometer message for the third sensor///////////////////
    accVec3.x = ax3;
    accVec3.y = ay3;
    accVec3.z = az3;

    gyroVec3.x = gx3;
    gyroVec3.y = gy3;
    gyroVec3.z = gz3;


      if (ref_val == false){
         base_val1 = sensor.getPressurePa();
         base_val2 = sensor2.getPressurePa();
         base_val3 = sensor3.getPressurePa();
         ref_val = true;
      //    counter = 0;
      }

       Last_val1[1] = sensor.getPressurePa();
       Last_val2[1] = sensor2.getPressurePa();
       Last_val3[1] = sensor3.getPressurePa();       
       
    if (abs(base_val1 - Last_val1[1]) > 18)
       touch = true;
       else if(abs(base_val2 - Last_val2[1]) > 18)
       touch = true;
       else if(abs(base_val3 - Last_val3[1]) > 18)
       touch = true;
       else
       touch = false;
       Serial.print(base_val1); Serial.print("\t"); Serial.println(Last_val1[1]);
      
      touch_msg.data  = touch;    
      pub_touch.publish(&touch_msg);   

/////////////Second touch/////////////////////
   if (abs(base_val1 - Last_val1[1]) > 360)
       touch2 = true;
       else if(abs(base_val2 - Last_val2[1]) > 360)
       touch2 = true;
       else if(abs(base_val3 - Last_val3[1]) > 360)
       touch2 = true;
       else
       touch2 = false;
      
      touch2_msg.data  = touch2;    
      pub_touch2.publish(&touch2_msg);      
    
    
   // release_msg.data  = release_item;  
   // pub_release.publish(&release_msg);
    
//    pub_pressure.publish(&sensor_pressure_msg);
//    pub_touch.publish(&touch_msg);
   // pub_acc1.publish(&accVec1);
    pub_gyro1.publish(&gyroVec1);
   // delay(1);
   // pub_acc2.publish(&accVec2);
     pub_gyro2.publish(&gyroVec2);
   // delay(1);
   // pub_acc3.publish(&accVec3);
   // pub_gyro3.publish(&gyroVec3);
    //delay(1);
  //  chatter.publish(&sensor_data);     
//
//       if (sensor_pressure_msg.x >= 99020.0 )
//       touch = true;
//       else if(sensor_pressure_msg.y >= 99020.0)
//       touch = true;
//       else if(sensor_pressure_msg.z >= 99020.0)
//       touch = true;
//       else
//       touch = false;
       
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
