/*
 *********************************************************************************
 * Author: Gorkem Anil Al
 * Email: gga31@bath.ac.uk
 * Date: 6-March-2020
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description:
 *
 * Requirements:
 * - Installation of SparkFun_APDS9960.h
 * - Installation of Adafruit_VL6180X.h
 * - Installation of SparkFun_VL6180X.h
 *********************************************************************************
 */

#include <Wire.h>
#include "SparkFun_APDS9960.h"
#include "Adafruit_VL6180X.h"
#include "SparkFun_VL6180X.h"

#define TCAADDR 0x70

// Proximity Sensors
//Adafruit_VL6180X v1 = Adafruit_VL6180X();
//Adafruit_VL6180X v2 = Adafruit_VL6180X();

#define VL6180X_ADDRESS 0x29
VL6180x sensor1(VL6180X_ADDRESS);
VL6180x sensor2(VL6180X_ADDRESS);
VL6180x sensor3(VL6180X_ADDRESS);

// Global Variables
SparkFun_APDS9960 apds1 = SparkFun_APDS9960();
SparkFun_APDS9960 apds2 = SparkFun_APDS9960();
SparkFun_APDS9960 apds3 = SparkFun_APDS9960();

int gesture[2];

int green = 2;
int yellow = 8;

char ReadReq;


void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup() {
     pinMode(green, OUTPUT); 
     pinMode(yellow, OUTPUT);
     Wire.begin();
     Serial.begin(9600);
    // Serial.println(F("SparkFun APDS-9960 - GestureTest"));

  /* Initialise the 1st sensor at channel 1*/
    
  tcaselect(0);
  
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds1.init() ) {
   // Serial.println(F("APDS-9960 initialization complete"));
  } else {
   // Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

    // Start running the APDS-9960 gesture sensor engine
  if ( apds1.enableGestureSensor(true) ) {
   // Serial.println(F("Gesture sensor is now running"));
  } else {
   // Serial.println(F("Something went wrong during gesture sensor init!"));
  }

  /* Initialise the 2nd sensor at channel 2*/
  
  tcaselect(1);
  
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds2.init() ) {
  //  Serial.println(F("APDS-9960 initialization complete"));
  } else {
   // Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

    // Start running the APDS-9960 gesture sensor engine
  if ( apds2.enableGestureSensor(true) ) {
    //Serial.println(F("Gesture sensor is now running"));
  } else {
    //Serial.println(F("Something went wrong during gesture sensor init!"));
  }

    tcaselect(2);
    //Serial.println("VL6180x Proximity Sensor Test"); Serial.println("");
    if(sensor1.VL6180xInit() != 0){
    //Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
  }; 
  sensor1.VL6180xDefautSettings(); //Load default settings to get started.
  
  tcaselect(3);
 // Serial.println("VL6180x Proximity Sensor Test"); Serial.println("");
    if(sensor2.VL6180xInit() != 0){
   // Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
  }; 
  sensor2.VL6180xDefautSettings(); //Load default settings to get started.
  
  tcaselect(4);
 // Serial.println("VL6180x Proximity Sensor Test"); Serial.println("");
    if(sensor3.VL6180xInit() != 0){
   // Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
  }; 
  sensor3.VL6180xDefautSettings(); //Load default settings to get started.
  
  
  for (int i = 0; i< 2; i++ )
    gesture[i] = 0;

}


void loop() {
    
      tcaselect(2);
     
   uint8_t range1 = sensor1.getDistance(); 

      tcaselect(3);
     
   uint8_t range2 = sensor2.getDistance();   

   tcaselect(4);
   uint8_t range3 = sensor3.getDistance();
   
   if (((range1 <= 250) && (range1 >= 200)) || ((range2 <= 250) && (range2 >= 200)) || ((range3 <= 250) && (range3 >= 200)) )
   {    
       digitalWrite(yellow, HIGH); 
      // for (int i = 0; i <=1; i++)
      // {
     //  tcaselect(i);   
      // gesture[i-1] = handleGesture(i);
    // Serial.println(gesture[i]);     
        //}
   }
   else{
    digitalWrite(yellow, LOW); 
    }
       if (((range1 <= 190) && (range1 >= 150)) || ((range2 <= 190) && (range2 >= 150)) || ((range3 <= 190) && (range3 >= 150)))
        {
                 digitalWrite(green, HIGH); 
      // for (int i = 0; i <2; i++)
      //  {
      // tcaselect(i);   
      // gesture[i] = handleGesture(i); // rawdata ledlerin durumunu gormek icin kapatildi. sonrasinda ac   
     //   }
         
        for (int i = 0; i <=1; i++)
      {
          tcaselect(i);   
          handleGesture(i+1);                    
      }
          
           //  tcaselect(0);   
           //  gesture[1] = handleGesture(1); 
           //   tcaselect(1);   
          //   gesture[2] = handleGesture(2);                
          }
          else
          {
            digitalWrite(green, LOW);
            }
 
}


int handleGesture(int value) {

int detectedValue = 0;

  if( value == 1 )
  {
    if ( apds1.isGestureAvailable() )
    {
     //Serial.print(-1); Serial.print(' ');
       ( apds1.readGesture() );
  }
  }
  else if(value == 2 )
  {
    if ( apds2.isGestureAvailable() )
    {
    //  Serial.print(-2); Serial.print(' ');
        Serial.print(','); Serial.print(' ');
     ( apds2.readGesture() );
      Serial.println(); // 2 satir kodu bir alt satira gecir
    }  
  }
  else
  {
    if ( apds3.isGestureAvailable() )
    {
    ( apds3.readGesture() );
    }
  }
  
    return detectedValue;
}
