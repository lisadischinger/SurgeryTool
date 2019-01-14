/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"
#include "math.h"
#include "AdvancedSerial.h"

#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
  //#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

// Pin definitions
  int intPin = 2;                           // These can be changed, 2 and 3 are the Arduinos ext int pins

// LMD variables
  bool StationairyRead = false;         // used to figure out the cutoff range; initially set to false just to test out the code given
  bool CalibrateNew = false;            // set true if you want to reclibrate the sensor
  int i;                                // counter to modify data arrays
  int array_size = 15;                    // the number of rows in the data matrix
  float imu_data[15][4];  //[7];         // array of the data gathered from the gyroscope and the accelerometer; [t, ax, ay, az]  gx, gy, gz]
  float vel[15][4];                      // array of [t, vx, vy, vz] absolute velocity
  float pos[15][4];                     // array of [t, x, y, z] absolute position
  unsigned int t_0;                     // this is the initial time used for integration purposes
  //int pos_0[3] = {0, 0, 0};             // initial position and velocity of the imu
  float imu1_x[3];                      // first IMU location data
  //float imu2_x[3];                    // second IMU location data
  //float imu_data_stat[15][7];         // arrays used to collect stationary data to find the range x_accel wanders
  float cutOff[6];                      // array that holds the ranges of the stationairy data
  float t;                              //  current time
    
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

/*
// For Graphing real time purposes
// https://github.com/Nick1787/AdvancedSerial/wiki
//Instantiate a new advanced serial object with 500 transmit slots
AdvancedSerial AdvSerial(&Serial, 2);
*/

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x6B);                             // PWR_MGMT_1 register
  Wire.write(0);                                // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
 
  /*
  //Add globals to the transmit queue.  The 3rd add signal command is ingored since there are only two slots
  AdvSerial.addSignal("time",&imu_data_stat[0]);
  AdvSerial.addSignal("stationairy ax",&imu_data_stat[1]);
  */

  while(!Serial){};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71)                                                    // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));
    if(CalibrateNew)
    {
      Serial.println(" starting to calibrate; move the IMU in a figure 8");
      delay(4000);                                                    // wait a second to let the user start to move the MPU for clibration
      myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);        // Calibrate gyro and accelerometers, load biases in bias registers
      Serial.println(" Calibration is done, thanks!");
      // record these biases so that leter on we don't always have to calibrate
      Serial.print("gyro_bias = ");
      for(int j = 0; j < 3-1; j++)         // incrament for the different columns
      {
        Serial.print(myIMU.gyroBias[j]);Serial.print(" ");
      }
      int j = 3-1;
      Serial.println(myIMU.gyroBias[j]);                // the last value of the row should start a new line
      Serial.print("accel_bias = ");
      for(int j = 0; j < 3-1; j++)         // incrament for the different columns
      {
        Serial.print(myIMU.accelBias[j]);Serial.print(" ");
      }
      Serial.println(myIMU.accelBias[j]);                // the last value of the row should start a new line
    }
    else
    {
      myIMU.gyroBias[0] = -0.10; myIMU.gyroBias[1] = 1.67; myIMU.gyroBias[2] = 6.95;      // averaged from 1.12.19 figure 8 calibration data
      myIMU.accelBias[0] = -0.13; myIMU.accelBias[1] = 0.07, myIMU.accelBias[2] = 0.00;
    }
    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println("MPU9250 initialized for active data mode....");
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    //Serial.println("AK8963 initialized for active data mode....");

    // Get sensor resolutions, only need to do this once
    myIMU.getAres(); myIMU.getGres(); myIMU.getMres();

    //DebugTest(WHO_AM_I_MPU9250, myIMU.selfTest, d, myIMU.factoryMagCalibration, myIMU.magBias, myIMU.magScale)

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x"); Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  
  //////////////////////////////
  // LMD Code
  //////////////////////////////
  
  t_0 = millis();                                   // This captures the initial time in milliseconds

  /*
  while (StationairyRead)                           // used to see the range at which the sensor oscilated while it sits stationairy
  {
    Serial.println("Please wait while initializing the IMUs");
    for (int i=0; i <= 20; i++)                   // collect 100 stationairy sample points
    {
      data_gather(imu_data_stat[i]);              // feeding it the array that it is to fix up and the counter to modify the array
      delay(1000);
    }
     
    ////////////
    Serial.println("imu_data_stat ");           // check what the array looks like
    for(int i = 0; i < sizeof(imu_data_stat)/sizeof(imu_data_stat[0]); i++)
    {
      Serial.print(imu_data_stat[i][0]); Serial.print(imu_data_stat[i][1]); Serial.print(imu_data_stat[i][2]); Serial.println(imu_data_stat[i][3]);
    }
    ///////////
    
    StationairyRead = false;                      // we no longer need to read to figure out how oscilates
    //cut_off(cutOff);                // calculate the range at which the sensor wobbles while sitting still
  }
  */
  
  Serial.println(" now you may move the IMU");
}

void loop()
{
  for(int a = 0; a < array_size; a++)
  {
    data_gather(imu_data[a]);     // gather new data from the specified imu
    delay(100);
    if(a>= 1);
    {
      integrate( t_0, vel[a-1], imu_data[a], imu_data[a-1], vel[a]);              // calculate the velocity of the specified imu
      integrate( t_0, pos[a-1], vel[a], vel[a-1], pos[a]);                        //calculate the location of the sensor relative to the starting point
    }
  }
  
  Serial.println("Accleration data [in/s^2]");
  print_array(imu_data, array_size, 4);
  Serial.println(" ");
  Serial.println("calculated velocity data [in/s]");
  print_array(vel, array_size, 4);
  Serial.println(" ");
  Serial.println("Position data [in]");
  print_array(pos, array_size, 4);
  Serial.println(" ");
}


///////////////////////////////////////////
// Lisa's Functions
///////////////////////////////////////////

void data_gather(float out_array[]) {
  // this is the code used to read and gather data from the specified IMU
  // it will return "data" which is the array of the float values gathered.
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt

  unsigned t_now;
  float g_to_ins2 = 386.05;                                    // 386.05[in/s^2]/ 1g
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    t_now = millis();
    t = (float)(t_now - t_0)/(float)1000;                           //time at which we are reading, convert to seconds

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (((float)myIMU.accelCount[0] * myIMU.aRes) - myIMU.accelBias[0]) * g_to_ins2 ; //unit conversion from [g] to [in/s^2]
    myIMU.ay = ((float)myIMU.accelCount[1] * myIMU.aRes) * g_to_ins2; // - myIMU.accelBias[1];
    myIMU.az = ((float)myIMU.accelCount[2] * myIMU.aRes) * g_to_ins2; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual radians per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes * (M_PI/180);           // LMD added the pi/180 to convert from deg/s to rad/s
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes * (M_PI/180);
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes * (M_PI/180);

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by aircraft orientation standards! Pass gyro rate as rad/s

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("acc (x, y, z): "); Serial.print(myIMU.ax); Serial.print("  "); Serial.print(myIMU.ay); Serial.print("  "); Serial.print(myIMU.az);
        Serial.println(" in/s^2 ");

        // Print gyro values in degree/sec
        Serial.print("gyro (x, y, z): "); Serial.print(myIMU.gx, 3);Serial.print("  "); Serial.print(myIMU.gy, 3);Serial.print("  "); Serial.print(myIMU.gz, 3);
        Serial.println(" rad/sec ");

        /*
        // Print mag values in degree/sec
        Serial.print("mag (x,y,z): "); Serial.print(1000*myIMU.mx); Serial.print("  "); Serial.print(1000*myIMU.my);Serial.print("  ");Serial.print(1000*myIMU.mz)
        Serial.println(" mG ");
        */

        // Put all data into one variable to be sent to functions
        out_array[0]= t;
        out_array[1]= myIMU.ax; out_array[2]= myIMU.ay; out_array[3]= myIMU.az; 
        //out_array[4]= myIMU.gx; out_array[5]= myIMU.gy; out_array[6]= myIMU.gx;
      }

      myIMU.count = millis();
    }
  }
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    //if (myIMU.delt_t > 500)

      if(SerialDebug)
      {
        Serial.print("acc (x, y, z) = ");  Serial.print(myIMU.ax); Serial.print("  "); Serial.print(myIMU.ay);Serial.print("  "); 
        Serial.print(myIMU.az); Serial.println(" g");

        Serial.print("gyro (x, y, z) = ");  Serial.print(myIMU.gx,2); Serial.print("  "); Serial.print(myIMU.gy);Serial.print("  "); 
        Serial.print(myIMU.gz); Serial.println(" rad/s");

        /*
        Serial.print("mag (x, y, z) = ");  Serial.print(myIMU.mx); Serial.print("  "); Serial.print(myIMU.my);Serial.print("  "); 
        Serial.print(myIMU.mz); Serial.println(" mg");
        */

        Serial.print("(q0, qx, qy, qz) = ");  Serial.print(*getQ()); Serial.print("  "); Serial.print(*(getQ() + 1)); Serial.print("  "); 
        Serial.print(*(getQ() + 2)); Serial.print("  "); Serial.println(*(getQ() + 3));
      }

// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise. Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll. For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is 8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      // Declination of Graff Hall Corvalis (44.567 N, 123.270 W) is 15° 2.58' East on 2019-1-12
      myIMU.yaw  -= 15;                 //8.5 from Spark Fun
      myIMU.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: "); Serial.print(myIMU.yaw, 2); Serial.print(", "); Serial.print(myIMU.pitch, 2); Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = "); Serial.print((float)myIMU.sumCount / myIMU.sum, 2); Serial.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;

      // Put all data into one variable to be sent to functions
      out_array[0]= t; 
      out_array[1]= myIMU.ax; out_array[2]= myIMU.ay; out_array[3]= myIMU.az;
      //out_array[4]= myIMU.yaw; out_array[5]= myIMU.pitch; out_array[6]= myIMU.roll;
  }
  return;
}

void print_array(float arrayx[][4], int rowSize, int columnSize)
{
  // this function will print out all the values within the array
  for(int i = 0; i < rowSize; i++)                // incrament for the different rows
  {
    for(int j = 0; j < columnSize-1; j++)         // incrament for the different columns
    {
      Serial.print(arrayx[i][j]);Serial.print(" ");
    }
    int j = columnSize - 1;
    Serial.println(arrayx[i][j]);                // the last value of the row should start a new line
  }
  return;
}

void integrate(unsigned int t_0, float vect_past[], float data_now[], float data_past[], float out_array[])
{
  // This function is used to iterate the information given; if acceleration is given then this caluates the new velocity,
  // if velocities are given then this calculates the new position
  float d_vect_rel[3];
  float dx[3] = {data_now[1] - data_past[1], data_now[2] - data_past[2], data_now[3] - data_past[3]};     // this calculates the differences in linear accleration of the two times
  float dt = data_now[0] - data_past[0];                                                    // calculate the time between samples [econds]
  
  for (int i = 0; i < 3; i++)                                                                   // multiply the array by a dt
    {
    d_vect_rel[i] = dt * dx[i];
    }
  out_array[0] = data_now[0];                                                          // this gives the time of sample now with the updated vector info
  out_array[1] = d_vect_rel[0] + vect_past[0];
  out_array[2] = d_vect_rel[1] + vect_past[1];
  out_array[3] = d_vect_rel[2] + vect_past[2];
  //Serial.println(" I made it to the calc function!");
  return;
}

/*
void cut_off(float out_array[]){
  // this function finds the ranges to cut off for the high pass filter
  // based off of the stationairy data. currently we will juts be finding the max and min and 
  //stating that that is the cuttoff range

  float ax_array[sizeof(imu_data_stat)], ay_array[sizeof(imu_data_stat)], az_array[sizeof(imu_data_stat)],
        gx_array[sizeof(imu_data_stat)], gy_array[sizeof(imu_data_stat)], gz_array[sizeof(imu_data_stat)];
  
  for (int i = 0; i<= sizeof(imu_data_stat)/sizeof(imu_data_stat[0]); i++)            // first issolate the different data sets
  {
    ax_array[i] = imu_data_stat[i][1];
    ay_array[i] = imu_data_stat[i][2];
    az_array[i] = imu_data_stat[i][3];
    gx_array[i] = imu_data_stat[i][4];
    gy_array[i] = imu_data_stat[i][5];
    gz_array[i] = imu_data_stat[i][6];
  }

  out_array[0] = {array_max(ax_array),array_min(ax_array)};
  out_array[1] = {array_max(ay_array),array_min(ay_array)};
  out_array[2] = {array_max(az_array),array_min(az_array)};
  out_array[3] = {array_max(gx_array),array_min(gx_array)};
  out_array[4] = {array_max(gy_array),array_min(gy_array)};
  out_array[5] = {array_max(gz_array),array_min(gz_array)};
  return;
}


float array_max(float array[]){
  // find the maximum value in an array
  
  int max_v = 0;                          // placeholder for max value
  for (int i=0; i< sizeof(array); i++)
  {
    max_v = max(array[i], max_v);
  }
  return max_v;
}


float array_min(float array[]){
  // find the minimum value in an array
  
  int min_v = 0;                          // placeholder for max value
  for (int i=0; i< sizeof(array); i++)
  {
    min_v = min(array[i], min_v);
  }
  return min_v;
}
*/

void high_pass(){
  // This filters the data from the specified imu and sees if it is actually moving. This is a gate 
  // that needs to be passed before going to the calculation, if not passed, go get new imu data
  return;
}

/*
void DebugTest(HEX WHO_AM_I_MPU9250, array selfTest,HEX d, array factoryMagCalibration, array magBias, array magScale){
  // This function is just meant to run all the initial debugging needed to chack that connection is working
  // beware the data types were hap hazardly selected...chossen to match how they were going to be printed

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.print(F("who I AM 0x"));
  Serial.print(WHO_AM_I_MPU9250, HEX);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  // Start by performing self test and reporting values
  myIMU.MPU9250SelfTest(myIMU.selfTest);
  Serial.print(F("x-axis self test: acceleration trim within : "));
  Serial.print(selfTest[0],1); Serial.println("% of factory value");
  Serial.print(F("y-axis self test: acceleration trim within : "));
  Serial.print(selfTest[1],1); Serial.println("% of factory value");
  Serial.print(F("z-axis self test: acceleration trim within : "));
  Serial.print(selfTest[2],1); Serial.println("% of factory value");
  Serial.print(F("x-axis self test: gyration trim within : "));
  Serial.print(selfTest[3],1); Serial.println("% of factory value");
  Serial.print(F("y-axis self test: gyration trim within : "));
  Serial.print(selfTest[4],1); Serial.println("% of factory value");
  Serial.print(F("z-axis self test: gyration trim within : "));
  Serial.print(selfTest[5],1); Serial.println("% of factory value");

  // Read the WHO_AM_I register of the magnetometer, this is a good test of
  // communication
  Serial.print("AK8963 ");
  Serial.print("I AM 0x");
  Serial.print(d, HEX);
  Serial.print(" I should be 0x");
  Serial.println(0x48, HEX);

  //  Serial.println("Calibration values: ");
  Serial.print("X-Axis factory sensitivity adjustment value ");
  Serial.println(factoryMagCalibration[0], 2);
  Serial.print("Y-Axis factory sensitivity adjustment value ");
  Serial.println(factoryMagCalibration[1], 2);
  Serial.print("Z-Axis factory sensitivity adjustment value ");
  Serial.println(factoryMagCalibration[2], 2);

  // The next call delays for 4 seconds, and then records about 15 seconds of
  // data to calculate bias and scale.
  //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
  Serial.println("AK8963 mag biases (mG)");
  Serial.println(magBias[0]);
  Serial.println(magBias[1]);
  Serial.println(magBias[2]);

  Serial.println("AK8963 mag scale (mG)");
  Serial.println(magScale[0]);
  Serial.println(magScale[1]);
  Serial.println(magScale[2]);
  //    delay(2000); // Add delay to see results before serial spew of data

  Serial.println("Magnetometer:");
  Serial.print("X-Axis sensitivity adjustment value ");
  Serial.println(factoryMagCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value ");
  Serial.println(factoryMagCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value ");
  Serial.println(factoryMagCalibration[2], 2);

  return;
}
*/
