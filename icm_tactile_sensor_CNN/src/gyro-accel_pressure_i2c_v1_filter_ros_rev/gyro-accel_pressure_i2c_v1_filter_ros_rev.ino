/******************************************************************
  ICP10xx library is used for the TDK InvenSense ICP-101xx
 series of barometric pressure sensors.

 Kalman filter was implemented to use roll and pitch values. 
 This code is ready to test with UR3 robot.
 
******************************************************************/
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
//#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
//#include <multimodal_tactile_sensor/icmDataMsg.h>

#include<Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <icp101xx.h> 

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

//Set up the ros node and publisher
   
   std_msgs::Float32 kal_Angle_X_msg;
   std_msgs::Float32 kal_Angle_Y_msg;
   std_msgs::Float32 temperature_msg;
   std_msgs::Float32 pressure_msg;
   std_msgs::Bool touch_msg;
   geometry_msgs::Vector3 accVec;
   geometry_msgs::Vector3 gyroVec;
   std_msgs::Float32 average_x_msg;
   std_msgs::Float32 average_p_msg;
   std_msgs::Float32 average_y_msg;

   ros::Publisher pub_averagex("averagey", &average_y_msg);
   ros::Publisher pub_averagep("averagep", &average_p_msg);
   ros::Publisher pub_averagey("averagex", &average_x_msg);
   ros::Publisher pub_rollangle("kalmanX", &kal_Angle_X_msg);
   ros::Publisher pub_pitchangle("kalmanY", &kal_Angle_Y_msg);
   ros::Publisher pub_temperature("temperature", &temperature_msg);
   ros::Publisher pub_pressure("pressure", &pressure_msg);
   ros::Publisher pub_acc("acc", &accVec);
   ros::Publisher pub_gyro("gyro", &gyroVec);
   ros::Publisher pub_touch("touch", &touch_msg); // first touch to feel the object
   
   ros::NodeHandle nh;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accelX, accelY, accelZ; //strore the values read from the sensor
double gForceX, gForceY, gForceZ; //use for the calculation of the gforce

double gyroX, gyroY, gyroZ;
double rotX, rotY, rotZ;

int count = 0;
bool touch = false;
float sample_size_x[20], sample_size_y[20], sample_size_p[20], avrg_x = 0.0, avrg_y = 0.0, avrg_p = 0.0;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint16_t load;
float output;

uint32_t timer;

long int pressure;
int temp_2;

// Sensor is an ICP101xx object
ICP101xx sensor;

#define I2C_ADDR 0x68
#define PRE_ADDR 0x63

void setup() {
sensor.begin(); //force sensor
Serial.begin(115200);
//Serial.begin(57600);
//while (! Serial); // Wait untilSerial is ready 
Wire.begin();
nh.initNode();
nh.advertise(pub_temperature);
nh.advertise(pub_pressure);
nh.advertise(pub_acc);
nh.advertise(pub_gyro);
nh.advertise(pub_rollangle);
nh.advertise(pub_pitchangle);
nh.advertise(pub_touch);
nh.advertise(pub_averagex);
nh.advertise(pub_averagey);
nh.advertise(pub_averagep);

//TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  setupICM();
  recordAccelRegisters();
  recordGyroRegisters();


  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accelY, accelZ) * RAD_TO_DEG;
  double pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;
  double pitch = atan2(-accelX, accelZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

long publisher_timer;

void loop() {

if (millis() > publisher_timer) {

  recordAccelRegisters();
  recordGyroRegisters();
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accelY, accelZ) * RAD_TO_DEG;
  double pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;
  double pitch = atan2(-accelX, accelZ) * RAD_TO_DEG;
#endif

 double rotX = gyroX / 131.0; //sensitivity scale factor for gyro
 double rotY = gyroY / 131.0;
  
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, rotX, dt); // Calculate the angle using a Kalman filter
    
  if (abs(kalAngleX) > 90)
    rotY = -rotY; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, rotY, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  }else
    kalAngleY = kalmanY.getAngle(pitch, rotY, dt); // Calculate the angle using a Kalman filter
     
  if (abs(kalAngleY) > 90)
    rotX = -rotX; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, rotX, dt); // Calculate the angle using a Kalman filter
#endif

  //  Serial.println(pitch);
  gyroXangle += rotX * dt; // Calculate gyro angle without any filter
  gyroYangle += rotY * dt;

 // gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
 // gyroYangle += kalmanY.getRate() * dt;
 
  compAngleX = 0.93 * (compAngleX + rotX * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + rotY * dt) + 0.07 * pitch;
  

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

//################## FORCE SENSOR #####################################
    // Optional: Measurement mode
    //    sensor.FAST: ~3ms
    //    sensor.NORMAL: ~7ms (default)
    //    sensor.ACCURATE: ~24ms
    //    sensor.VERY_ACCURATE: ~95ms
    sensor.measure(sensor.ACCURATE);
    rosData();

 /* Print Data */  
 //  printData(); 
  /* delay(2); */
    //}
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
  processAccelData(); 
    }

void processAccelData(){
  gForceX = accelX / 16384.0;  //sensitivity scale factor
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
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
  //processGyroData(); 
  }
/*
void processGyroData(){
  rotX = gyroX / 131.0; //sensitivity scale factor for gyro
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
   }  
*/
void rosData() {
    kal_Angle_X_msg.data = kalAngleX;
    kal_Angle_Y_msg.data = kalAngleY;
    temperature_msg.data = sensor.getTemperatureC();
    pressure_msg.data  = sensor.getPressurePa();
    touch_msg.data  = touch; 
    
    accVec.x = accelX;
    accVec.y = accelY;
    accVec.z = accelZ;

    gyroVec.x = gyroX;
    gyroVec.y = gyroY;
    gyroVec.z = gyroZ;

    sample_size_x[count] = kalAngleX;
    sample_size_y[count] = kalAngleY;
    sample_size_p[count] = sensor.getPressurePa();
    count++;

     if (count == 20)  //20 samples are collected
      {
        for (int i = 0; i < 20; i++)
        {
          avrg_x = avrg_x + sample_size_x[i];   //all 20 samples are added
          avrg_y = avrg_y + sample_size_y[i];   //all 20 samples are added
          avrg_p = avrg_p + sample_size_p[i];   //all 20 samples are added
        }
        avrg_x = avrg_x / 20;
        avrg_y = avrg_y / 20;
        avrg_p = avrg_p / 21;
                      
        count = 0;
      }   

      // if ((abs(avrg_x- kalAngleX) > 2.3) || (abs(avrg_y - kalAngleY) > 2.3) || (abs(avrg_p - sensor.getPressurePa()) > 500))
       if ((abs(avrg_p - sensor.getPressurePa()) > 500))
        touch = true;
      else
        touch = false;

    average_x_msg.data = avrg_x;
    pub_averagex.publish(&average_x_msg);
    
    average_y_msg.data = avrg_y;
    pub_averagey.publish(&average_y_msg);
    
    average_p_msg.data = avrg_p;
    pub_averagep.publish(&average_p_msg);
    
    pub_touch.publish(&touch_msg);     
    pub_rollangle.publish(&kal_Angle_X_msg);
    pub_pitchangle.publish(&kal_Angle_Y_msg);
    pub_temperature.publish(&temperature_msg);
    pub_pressure.publish(&pressure_msg);
    pub_acc.publish(&accVec);
    pub_gyro.publish(&gyroVec);
    
    publisher_timer = millis() + 1; //publish once a second

     avrg_x = 0;
     avrg_y = 0;
     //avrg_p = 0;
}
  /*
void printData() {
#if 0 // Set to 1 to activate
  Serial.print(";");
  Serial.print(accelX); Serial.print(";");
  Serial.print(accelY); Serial.print(";");
  Serial.print(accelZ); Serial.print(";");

  Serial.print(gyroX); Serial.print(";");
  Serial.print(gyroY); Serial.print(";");
  Serial.print(gyroZ); Serial.print(";");

#endif

 // Serial.print(roll); Serial.print("\t");
//  Serial.print(gyroXangle); Serial.print("\t");
//  Serial.print(compAngleX); Serial.print("\t");

//  Serial.print("Angle_x: ");
  Serial.print(kalAngleX); Serial.print(";");

 // Serial.print("\t");
//
 // Serial.print(pitch); Serial.println("\t");
//  Serial.print(gyroYangle); Serial.print("\t");
//  Serial.print(compAngleY); Serial.print("\t");

//  Serial.print("\t\t\tAngle_y: ");

  Serial.print(kalAngleY); Serial.print(";");
  
//  Serial.print("\t");
//  Serial.print("\t\t\tPressure: ");

  Serial.print(sensor.getPressurePa()); Serial.print(";");
  Serial.println(sensor.getTemperatureC());
  
}
*/
