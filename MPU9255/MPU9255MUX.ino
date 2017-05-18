#include <Wire.h>
//#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

#define TCAADDR     0x70

MPU9250 myIMU[3];



void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup(void) 
{
  Wire.begin();
  Serial.begin(38400);
  Serial.println("MPU9255 Test"); Serial.println("");
  
  /* Initialise the 1st sensor */
  tcaselect(0);
  initializeMPU(0);
  
  /* Initialise the 2nd sensor */
  tcaselect(1);
  initializeMPU(1);

  /* Initialise the 3nd sensor */
  //tcaselect(2);
  //initializeMPU(2);


  // User environmental x-axis correction in milliGauss, should be
  // automatically calculated
  myIMU[0].magbias[0] = -192.1830  ;
  // User environmental x-axis correction in milliGauss TODO axis??
  myIMU[0].magbias[1] = +619.5650 ;
  // User environmental x-axis correction in milliGauss
  myIMU[0].magbias[2] = -759.0226;
  
  // User environmental x-axis correction in milliGauss, should be
  // automatically calculated
  myIMU[1].magbias[0] = +81.6619  ;
  // User environmental x-axis correction in milliGauss TODO axis??
  myIMU[1].magbias[1] = +529.9864 ;
  // User environmental x-axis correction in milliGauss
  myIMU[1].magbias[2] = -729.7883 ;

    // User environmental x-axis correction in milliGauss, should be
  // automatically calculated
  myIMU[2].magbias[0] = -63.4139  ;
  // User environmental x-axis correction in milliGauss TODO axis??
  myIMU[2].magbias[1] = +119.1866  ;
  // User environmental x-axis correction in milliGauss
  myIMU[2].magbias[2] = -95.6796;
  
}

void loop(void) 
{
  tcaselect(0);
  readMPU(0);

  
  tcaselect(1);
  readMPU(1);

  //tcaselect(2);
  //readMPU(2);
}

void initializeMPU(int i)
{
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU[i].readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);                 //MPU9250_ADDRESS is 0x68, WHO_AM_I_MPU9250 is 0x75
  Serial.print(MPU9250_ADDRESS,HEX);Serial.print("    ");Serial.println(WHO_AM_I_MPU9250,HEX);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x73, HEX);

  if (c == 0x73)                                                              // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU[i].MPU9250SelfTest(myIMU[i].SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU[i].SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU[i].SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU[i].SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU[i].SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU[i].SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU[i].SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU[i].calibrateMPU9250(myIMU[i].gyroBias, myIMU[i].accelBias);

    myIMU[i].initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU[i].readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }
    // Get magnetometer calibration from AK8963 ROM
    myIMU[i].initAK8963(myIMU[i].magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU[i].magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU[i].magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU[i].magCalibration[2], 2);
    }

  } // if (c == 0x73)
 else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    
    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  } 
}



void readMPU(int i)
{
  if (myIMU[i].readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU[i].readAccelData(myIMU[i].accelCount);  // Read the x/y/z adc values
    myIMU[i].getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU[i].ax = (float)myIMU[i].accelCount[0]*myIMU[i].aRes; // - accelBias[0];
    myIMU[i].ay = (float)myIMU[i].accelCount[1]*myIMU[i].aRes; // - accelBias[1];
    myIMU[i].az = (float)myIMU[i].accelCount[2]*myIMU[i].aRes; // - accelBias[2];

    myIMU[i].readGyroData(myIMU[i].gyroCount);  // Read the x/y/z adc values
    myIMU[i].getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU[i].gx = (float)myIMU[i].gyroCount[0]*myIMU[i].gRes;
    myIMU[i].gy = (float)myIMU[i].gyroCount[1]*myIMU[i].gRes;
    myIMU[i].gz = (float)myIMU[i].gyroCount[2]*myIMU[i].gRes;

    myIMU[i].readMagData(myIMU[i].magCount);  // Read the x/y/z adc values
    myIMU[i].getMres();


    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU[i].mx = (float)myIMU[i].magCount[0]*myIMU[i].mRes*myIMU[i].magCalibration[0] -
               myIMU[i].magbias[0];
    myIMU[i].my = (float)myIMU[i].magCount[1]*myIMU[i].mRes*myIMU[i].magCalibration[1] -
               myIMU[i].magbias[1];
    myIMU[i].mz = (float)myIMU[i].magCount[2]*myIMU[i].mRes*myIMU[i].magCalibration[2] -
               myIMU[i].magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  /*Serial.print("IMU No.");
  Serial.println(i);
  Serial.println(myIMU[0].ax);
  Serial.println(myIMU[1].ax);
  */
  // Must be called before updating quaternions!
  myIMU[i].updateTime();
  myIMU[i].MadgwickQuaternionUpdate(myIMU[i].ax, myIMU[i].ay, myIMU[i].az, myIMU[i].gx*PI/180.0f, 
                          myIMU[i].gy*PI/180.0f, myIMU[i].gz*PI/180.0f,  myIMU[i].my,  
                          myIMU[i].mx, -myIMU[i].mz, myIMU[i].deltat);
  //myIMU[i].MahonyQuaternionUpdate(myIMU[i].ax, myIMU[i].ay, myIMU[i].az, myIMU[i].gx*DEG_TO_RAD,
  //                       myIMU[i].gy*DEG_TO_RAD, myIMU[i].gz*DEG_TO_RAD, myIMU[i].my,
  //                       myIMU[i].mx, -myIMU[i].mz, myIMU[i].deltat);

 /*
  Serial.println(*myIMU[i].getQ());
  Serial.println(myIMU[i].q0);
  */
  
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU[i].delt_t = millis() - myIMU[i].count;
    if (myIMU[i].delt_t > 500)
    {
    /*  
    Serial.print("millis: ");
    Serial.println(millis());
    
    Serial.print("delt_t: ");
    Serial.println(myIMU[i].delt_t);¸
    
    Serial.print("deltat: ");
    Serial.println(myIMU[i].deltat);
    */
      if(SerialDebug)
      {
        /*Serial.print("IMU No.");
        Serial.println(i);
        Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = "); Serial.print( myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print( myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print( myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = "); Serial.print( (int)myIMU.mx );
        Serial.print(" my = "); Serial.print( (int)myIMU.my );
        Serial.print(" mz = "); Serial.print( (int)myIMU.mz );
        Serial.println(" mG");
        
        Serial.print("q0 = "); Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
        */
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      /*myIMU[i].yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU[i].pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU[i].roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU[i].pitch *= RAD_TO_DEG;
      myIMU[i].yaw   *= RAD_TO_DEG;
      
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      
      myIMU[i].yaw   -= 8.5;
      myIMU[i].roll  *= RAD_TO_DEG;
      */
      float R[3][3];
      R[0][0]=1.0f-2.0f*(myIMU[i].qy * myIMU[i].qy + myIMU[i].qz * myIMU[i].qz);
      R[0][1]=2*(myIMU[i].qx * myIMU[i].qy - myIMU[i].q0 * myIMU[i].qz);
      R[0][2]=2*(myIMU[i].qx * myIMU[i].qz + myIMU[i].q0 * myIMU[i].qy);
      R[1][0]=2*(myIMU[i].qx * myIMU[i].qy + myIMU[i].q0 * myIMU[i].qz);
      R[1][1]=1-2*(myIMU[i].qx * myIMU[i].qx + myIMU[i].qz * myIMU[i].qz);
      R[1][2]=2*(myIMU[i].qy * myIMU[i].qz - myIMU[i].q0 * myIMU[i].qx);
      R[2][0]=2*(myIMU[i].qx * myIMU[i].qz - myIMU[i].q0 * myIMU[i].qy);
      R[2][1]=2*(myIMU[i].q0 * myIMU[i].qx + myIMU[i].qy * myIMU[i].qz);
      R[2][2]=1-2*(myIMU[i].qx * myIMU[i].qx + myIMU[i].qy * myIMU[i].qy);
      
      if(SerialDebug)
      {
        /*Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
        Serial.println(" Hz");
        */
        Serial.print("IMU No.");
        Serial.println(i);
        Serial.print("q0=");
        Serial.print(myIMU[i].q0);
        Serial.print(",qx=");
        Serial.print(myIMU[i].qx);
        Serial.print(",qy=");
        Serial.print(myIMU[i].qy);
        Serial.print(",qz=");
        Serial.println(myIMU[i].qz);
        for(int k=0;k<3;k++){
          for(int j=0;j<2;j++)
          {
            Serial.print(R[k][j]);
            Serial.print(",");
          }
          Serial.print(R[k][2]);
          Serial.println(";");
        }
          
        
      }
      myIMU[i].count = millis();
      myIMU[i].sumCount = 0;
      myIMU[i].sum = 0;
    } // if (myIMU.delt_t > 500)
  
}
  
