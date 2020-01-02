/*
 * Copyright (C) 2019 Pablo Bernal-Polo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


//#define TEST

#define PACKAGE_SIZE 22
#define PACKAGE_SIZE_P 10
#define INFORMATION_SOURCE_CLASS_ID 30  // 10: GPS,  20: Velocimeter,  30: IMU
#define INFORMATION_SOURCE_CLASS_ID_P 100  // 10: GPS,  20: Velocimeter,  30: IMU
#define WHO_AM_I 11  // DO NOT CHANGE!!!
#define WHO_AM_I_P 101
#define IMU_MEASUREMENTS_PER_PRESSURE_MEASUREMENTS 8  // c*( 1/558 + 1/1766 + 1/486 ) = ( 1/166 + 1/36 )

#include "I2Cdev.h"
#include "MPU6050.h"  // accelerometer + gyroscope
#include <Wire.h>
#include <HMC5883L.h>  // magnetometer
#include <Adafruit_BMP085.h>  // pressure sensor
#include "MessageManager.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
HMC5883L magnetometer;
Adafruit_BMP085 pressureSensor;

MessageManager MM( 2 );

// we will save measurements here
int16_t amx, amy, amz;
int16_t wmx, wmy, wmz;
int16_t tempAW;
int16_t mmx, mmy, mmz;
uint32_t tempP;
uint32_t pm;

int cm;

void setup() {
  // we start the serial connection
  Serial.begin(115200);

  initialize_wire();
  
  initialize_MPU6050();

  initialize_magnetometer();

  initialize_pressureSensor();

  cm = 0;
}


void loop() {
  // we take measurements
  accelgyro.getMotion6( &amx , &amy , &amz , &wmx , &wmy , &wmz );  // 558 Hz
  tempAW = accelgyro.getTemperature();  // 1766 Hz
  Vector mf = magnetometer.readRaw();  // 488 Hz
  mmx = mf.XAxis;
  mmy = mf.YAxis;
  mmz = mf.ZAxis;
  cm++;
  if( cm > IMU_MEASUREMENTS_PER_PRESSURE_MEASUREMENTS ){
    tempP = pressureSensor.readRawTemperature();  // 166 Hz
    pm = pressureSensor.readRawPressure();  // 36 Hz
  }
  
#if !defined TEST
  // we send the package ( each packet is a 8-bit signed two's complement integer )
  //  first, we prepare the package who_am_i + (7 data(acceleration + angular velocity + temperature))*(2 byte/data) + 2 checksum
  const int8_t packet[PACKAGE_SIZE] = { INFORMATION_SOURCE_CLASS_ID , WHO_AM_I ,
                                        (int8_t)amx , (int8_t)(amx >> 8) ,
                                        (int8_t)amy , (int8_t)(amy >> 8) ,
                                        (int8_t)amz , (int8_t)(amz >> 8) ,
                                        (int8_t)wmx , (int8_t)(wmx >> 8) ,
                                        (int8_t)wmy , (int8_t)(wmy >> 8) ,
                                        (int8_t)wmz , (int8_t)(wmz >> 8) ,
                                        (int8_t)tempAW , (int8_t)(tempAW >> 8) ,
                                        (int8_t)mmx , (int8_t)(mmx >> 8) ,
                                        (int8_t)mmy , (int8_t)(mmy >> 8) ,
                                        (int8_t)mmz , (int8_t)(mmz >> 8) };
  
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( PACKAGE_SIZE , packet );
  
  // we send the communication
  for(int i=0; i<PACKAGE_SIZE+4; i++){
    Serial.write( toWrite[i] );
  }
  
  if( cm > IMU_MEASUREMENTS_PER_PRESSURE_MEASUREMENTS ){
    int8_t* packetsTP = (int8_t*) &tempP;
    int8_t* packetsP = (int8_t*) &pm;
    const int8_t packetP[PACKAGE_SIZE_P] = { INFORMATION_SOURCE_CLASS_ID_P , WHO_AM_I_P ,
                                             packetsTP[0] , packetsTP[1] , packetsTP[2] , packetsTP[3] ,
                                             packetsP[0] , packetsP[1] , packetsP[2] , packetsP[3] };
    
    // we prepare the message
    int8_t* toWriteP = MM.prepare_message( PACKAGE_SIZE_P , packetP );
    
    // we send the communication
    for(int i=0; i<PACKAGE_SIZE_P+4; i++){
      Serial.write( toWriteP[i] );
    }
    cm = 0;
  }
  
#else
  /*Serial.print( amx );  Serial.print( " " );
  Serial.print( amy );  Serial.print( " " );
  Serial.print( amz );  Serial.print( " " );
  Serial.print( wmx );  Serial.print( " " );
  Serial.print( wmy );  Serial.print( " " );
  Serial.print( wmz );  Serial.print( " " );
  Serial.print( tempAW );  Serial.print( " " );
  Serial.print( mmx );  Serial.print( " " );
  Serial.print( mmy );  Serial.print( " " );
  Serial.print( mmz );  Serial.print( " " );
  Serial.print( tempP );  Serial.print( " " );*/
  Serial.println( pm );
#endif
}

void initialize_wire() {
  // we start the connection with the sensor
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  TWBR = 12;
}

void initialize_MPU6050() {
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // set range
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void initialize_magnetometer() {
  // Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!magnetometer.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  
  // Set measurement range
  // +/- 0.88 Ga: HMC5883L_RANGE_0_88GA
  // +/- 1.30 Ga: HMC5883L_RANGE_1_3GA (default)
  // +/- 1.90 Ga: HMC5883L_RANGE_1_9GA
  // +/- 2.50 Ga: HMC5883L_RANGE_2_5GA
  // +/- 4.00 Ga: HMC5883L_RANGE_4GA
  // +/- 4.70 Ga: HMC5883L_RANGE_4_7GA
  // +/- 5.60 Ga: HMC5883L_RANGE_5_6GA
  // +/- 8.10 Ga: HMC5883L_RANGE_8_1GA
  magnetometer.setRange(HMC5883L_RANGE_8_1GA);

  // Set measurement mode
  // Idle mode:              HMC5883L_IDLE
  // Single-Measurement:     HMC5883L_SINGLE
  // Continuous-Measurement: HMC5883L_CONTINOUS (default)
  magnetometer.setMeasurementMode(HMC5883L_CONTINOUS);
 
  // Set data rate
  //  0.75Hz: HMC5883L_DATARATE_0_75HZ
  //  1.50Hz: HMC5883L_DATARATE_1_5HZ
  //  3.00Hz: HMC5883L_DATARATE_3HZ
  //  7.50Hz: HMC5883L_DATARATE_7_50HZ
  // 15.00Hz: HMC5883L_DATARATE_15HZ (default)
  // 30.00Hz: HMC5883L_DATARATE_30HZ
  // 75.00Hz: HMC5883L_DATARATE_75HZ
  magnetometer.setDataRate(HMC5883L_DATARATE_75HZ);

  // Set number of samples averaged
  // 1 sample:  HMC5883L_SAMPLES_1 (default)
  // 2 samples: HMC5883L_SAMPLES_2
  // 4 samples: HMC5883L_SAMPLES_4
  // 8 samples: HMC5883L_SAMPLES_8
  magnetometer.setSamples(HMC5883L_SAMPLES_1);
}

void initialize_pressureSensor() {
  if (!pressureSensor.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}

